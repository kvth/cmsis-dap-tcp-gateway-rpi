/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fused mux+connect(+RTT) vendor commands. See node_query.h for the wire
 * format. This is an orchestrator, not a reimplementation: every step below
 * hands a synthetic sub-request to the same handler a standalone client
 * would call (swdmux::select_pos(), dp::handle_command(), rtt::handle_command()),
 * so there is exactly one place that knows how to do each of those things.
 */

#include "node_query.h"

#include <string.h>
#include <unistd.h>

#include "dp_connect.h"
#include "rtt.h"
#include "swdmux.h"
#include "vendor.h"

namespace node_query {

using namespace vendor;

namespace {

// cmd, flags, mux_pos, targetsel, ap, cb_addr, down_channel, up_channel,
// max_wait_ms, request_count.
constexpr size_t QUERY_REQUEST_HEADER_LEN =
    1U + 1U + 1U + 4U + 1U + 4U + 1U + 1U + 2U + 1U;
// cmd, status, dpidr, ctrl_stat, response_count.
constexpr size_t QUERY_RESPONSE_HEADER_LEN = 1U + 1U + 4U + 4U + 1U;
// Bytes each completed sub-response's fixed part occupies: write_accepted,
// read_len.
constexpr size_t SUB_RESPONSE_FIXED_LEN = 2U + 2U;

// cmd, flags, mux_pos, targetsel.
constexpr size_t DETECT_REQUEST_LEN = 1U + 1U + 1U + 4U;
// cmd, status, dpidr, ctrl_stat.
constexpr size_t DETECT_RESPONSE_LEN = 1U + 1U + 4U + 4U;

// However long a request asks to wait for a reply, this is the most the
// gateway's one client gets kept waiting on a single sub-request's read.
constexpr uint16_t MAX_WAIT_MS_CEILING = 500U;
// 1ms was a conservative first guess; measured round trips came back at
// 2-4ms for a real reply, so a coarse poll risks waiting most of a whole
// interval past a reply that was already sitting there. Finer polling only
// adds a handful of cheap SWD reads to the wait window, never removes work.
constexpr useconds_t POLL_INTERVAL_US = 200U;

// Sanity bound on request_count, not a buffer size: well above a typical
// per-cycle queue depth (order 10 requests drained per node per cycle),
// just enough headroom that a real batch is never anywhere near it. Keeps a
// single command from being able to lock up the gateway's one client for an
// unbounded time (each sub-request can wait up to MAX_WAIT_MS_CEILING).
constexpr uint8_t MAX_REQUESTS = 32U;

// Room for the largest sub-request/response this builds: an RTT_Write's
// [id, channel, len] header plus up to a full packet of payload, or an
// RTT_Read response of the same size.
constexpr size_t SCRATCH_LEN = 1024U;

uint32_t load_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint16_t load_le16(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint8_t *store_le16(uint8_t *p, uint16_t v) {
  *p++ = (uint8_t)v;
  *p++ = (uint8_t)(v >> 8);
  return p;
}

uint8_t *store_le32(uint8_t *p, uint32_t v) {
  *p++ = (uint8_t)v;
  *p++ = (uint8_t)(v >> 8);
  *p++ = (uint8_t)(v >> 16);
  *p++ = (uint8_t)(v >> 24);
  return p;
}

// Shared by Node_Query and Node_Detect: mux select (unless flags bit3 says
// to skip it) then DP connect. dpidr/ctrl_stat are zeroed on any failure.
Status detect(uint8_t flags, uint8_t mux_pos, uint32_t targetsel,
              uint32_t *dpidr, uint32_t *ctrl_stat) {
  *dpidr = 0U;
  *ctrl_stat = 0U;

  if ((flags & 0x08U) == 0U) {
    if (!swdmux::select_pos(mux_pos)) {
      return STATUS_MUX_FAILED;
    }
  }

  uint8_t req[6];
  uint8_t resp[10];
  req[0] = CMD_DP_CONNECT;
  req[1] = flags;
  store_le32(req + 2, targetsel);
  dp::handle_command(req, resp, 6U);

  const Status status = (Status)resp[1];
  if (status == STATUS_OK) {
    *dpidr = load_le32(resp + 2);
    *ctrl_stat = load_le32(resp + 6);
  }
  return status;
}

struct SubRequest {
  const uint8_t *data;
  uint16_t len;
};

uint32_t handle_detect_command(const uint8_t *request, uint8_t *response,
                               size_t request_room) {
  response[0] = CMD_NODE_DETECT;

  if (request_room < DETECT_REQUEST_LEN) {
    response[1] = STATUS_BAD_REQUEST;
    memset(response + 2, 0, DETECT_RESPONSE_LEN - 2U);
    return ((uint32_t)request_room << 16) | (uint32_t)DETECT_RESPONSE_LEN;
  }

  const uint8_t  flags     = request[1];
  const uint8_t  mux_pos   = request[2];
  const uint32_t targetsel = load_le32(request + 3);

  uint32_t dpidr = 0U, ctrl_stat = 0U;
  const Status status = detect(flags, mux_pos, targetsel, &dpidr, &ctrl_stat);

  response[1] = (uint8_t)status;
  uint8_t *body = response + 2;
  body = store_le32(body, dpidr);
  store_le32(body, ctrl_stat);

  return ((uint32_t)DETECT_REQUEST_LEN << 16) | (uint32_t)DETECT_RESPONSE_LEN;
}

uint32_t handle_query_command(const uint8_t *request, uint8_t *response,
                              size_t request_room, size_t response_room) {
  response[0] = CMD_NODE_QUERY;

  if (request_room < QUERY_REQUEST_HEADER_LEN) {
    response[1] = STATUS_BAD_REQUEST;
    memset(response + 2, 0, QUERY_RESPONSE_HEADER_LEN - 2U);
    return ((uint32_t)request_room << 16) | (uint32_t)QUERY_RESPONSE_HEADER_LEN;
  }

  const uint8_t  flags        = request[1];
  const uint8_t  mux_pos      = request[2];
  const uint32_t targetsel    = load_le32(request + 3);
  const uint8_t  ap           = request[7];
  const uint32_t cb_addr      = load_le32(request + 8);
  const uint8_t  down_channel = request[12];
  const uint8_t  up_channel   = request[13];
  uint16_t       max_wait_ms  = load_le16(request + 14);
  const uint8_t  request_count = request[16];

  if (max_wait_ms > MAX_WAIT_MS_CEILING) {
    max_wait_ms = MAX_WAIT_MS_CEILING;
  }

  if (request_count > MAX_REQUESTS) {
    response[1] = STATUS_BAD_REQUEST;
    memset(response + 2, 0, QUERY_RESPONSE_HEADER_LEN - 2U);
    return ((uint32_t)QUERY_REQUEST_HEADER_LEN << 16) | (uint32_t)QUERY_RESPONSE_HEADER_LEN;
  }

  // Walk the sub-request list once, up front: every length claim has to
  // stay inside the packet, a length that runs past the end is a malformed
  // request rather than a short read of whatever memory follows.
  SubRequest sub_requests[MAX_REQUESTS];
  size_t pos = QUERY_REQUEST_HEADER_LEN;
  for (uint8_t i = 0; i < request_count; i++) {
    if ((pos + 2U) > request_room) {
      response[1] = STATUS_BAD_REQUEST;
      memset(response + 2, 0, QUERY_RESPONSE_HEADER_LEN - 2U);
      return ((uint32_t)request_room << 16) | (uint32_t)QUERY_RESPONSE_HEADER_LEN;
    }
    const uint16_t len = load_le16(request + pos);
    pos += 2U;
    if ((pos + len) > request_room) {
      response[1] = STATUS_BAD_REQUEST;
      memset(response + 2, 0, QUERY_RESPONSE_HEADER_LEN - 2U);
      return ((uint32_t)request_room << 16) | (uint32_t)QUERY_RESPONSE_HEADER_LEN;
    }
    sub_requests[i].data = request + pos;
    sub_requests[i].len  = len;
    pos += len;
  }
  const uint32_t request_len = (uint32_t)pos;

  uint32_t dpidr = 0U, ctrl_stat = 0U;
  Status status = detect(flags, mux_pos, targetsel, &dpidr, &ctrl_stat);

  uint8_t response_count = 0U;
  size_t  body_used      = 0U;  // bytes written after QUERY_RESPONSE_HEADER_LEN

  if ((status == STATUS_OK) && (request_count > 0U)) {
    uint8_t scratch_req[SCRATCH_LEN];
    uint8_t scratch_resp[SCRATCH_LEN];

    // RTT_Start once for the whole batch, same shortcut as a single
    // request always took: cb_addr taken as exact, no search.
    scratch_req[0] = CMD_RTT_START;
    scratch_req[1] = ap;
    store_le32(scratch_req + 2, cb_addr);
    store_le32(scratch_req + 6, 0U);
    rtt::handle_command(scratch_req, scratch_resp, 10U, SCRATCH_LEN);
    status = (Status)scratch_resp[1];

    for (uint8_t i = 0; (status == STATUS_OK) && (i < request_count); i++) {
      const uint16_t want = sub_requests[i].len;
      if (want > (SCRATCH_LEN - 4U)) {
        status = STATUS_BAD_REQUEST;
        break;
      }

      // RTT_Write, looped: one call only ever takes what currently fits in
      // the ring. A stall (nothing accepted) fails just this sub-request's
      // read below, rather than the whole batch -- only a status other than
      // STATUS_OK from the write itself is treated as a hard failure.
      uint16_t write_accepted = 0U;
      while (write_accepted < want) {
        const uint16_t remaining = (uint16_t)(want - write_accepted);
        scratch_req[0] = CMD_RTT_WRITE;
        scratch_req[1] = down_channel;
        store_le16(scratch_req + 2, remaining);
        memcpy(scratch_req + 4, sub_requests[i].data + write_accepted, remaining);

        rtt::handle_command(scratch_req, scratch_resp, 4U + remaining, SCRATCH_LEN);
        status = (Status)scratch_resp[1];
        if (status != STATUS_OK) {
          break;
        }
        const uint16_t accepted = load_le16(scratch_resp + 2);
        write_accepted = (uint16_t)(write_accepted + accepted);
        if (accepted == 0U) {
          usleep(POLL_INTERVAL_US);
          break;
        }
      }
      if (status != STATUS_OK) {
        break;  // transport failure: abort the rest of the batch
      }

      // RTT_Read, polled until something shows up or the budget runs out --
      // skipped if the write above did not fully land, same as a
      // single-request Node_Query always skipped it.
      uint16_t read_len = 0U;
      if (write_accepted == want) {
        const uint16_t max_read = (uint16_t)(SCRATCH_LEN - 4U);
        const unsigned budget_us = (unsigned)max_wait_ms * 1000U;
        unsigned elapsed_us = 0U;
        for (;;) {
          scratch_req[0] = CMD_RTT_READ;
          scratch_req[1] = up_channel;
          store_le16(scratch_req + 2, max_read);
          rtt::handle_command(scratch_req, scratch_resp, 4U, SCRATCH_LEN);
          status = (Status)scratch_resp[1];
          if (status != STATUS_OK) {
            break;
          }
          read_len = load_le16(scratch_resp + 2);
          if ((read_len != 0U) || (elapsed_us >= budget_us)) {
            break;
          }
          usleep(POLL_INTERVAL_US);
          elapsed_us += POLL_INTERVAL_US;
        }
        if (status != STATUS_OK) {
          break;  // transport failure: abort the rest of the batch
        }
      }

      // [write_accepted][read_len][read_data], clamped to what is left of
      // the response packet: a batch whose replies do not all fit gets
      // fewer completed sub-responses rather than one the client would
      // drop whole.
      const size_t room = (response_room > (QUERY_RESPONSE_HEADER_LEN + body_used))
                              ? (response_room - QUERY_RESPONSE_HEADER_LEN - body_used)
                              : 0U;
      if (room < SUB_RESPONSE_FIXED_LEN) {
        break;
      }
      uint16_t clamped_read_len = read_len;
      if (clamped_read_len > (room - SUB_RESPONSE_FIXED_LEN)) {
        clamped_read_len = (uint16_t)(room - SUB_RESPONSE_FIXED_LEN);
      }

      uint8_t *entry = response + QUERY_RESPONSE_HEADER_LEN + body_used;
      entry = store_le16(entry, write_accepted);
      entry = store_le16(entry, clamped_read_len);
      if (clamped_read_len != 0U) {
        memcpy(entry, scratch_resp + 4, clamped_read_len);
      }
      body_used += SUB_RESPONSE_FIXED_LEN + clamped_read_len;
      response_count++;

      if (clamped_read_len < read_len) {
        break;  // out of response room; do not claim more sub-responses fit
      }
    }
  }

  response[1] = (uint8_t)status;
  uint8_t *body = response + 2;
  body = store_le32(body, dpidr);
  body = store_le32(body, ctrl_stat);
  *body = response_count;

  return (request_len << 16) | (uint32_t)(QUERY_RESPONSE_HEADER_LEN + body_used);
}

}  // namespace

bool handles(uint8_t cmd) {
  return (cmd == CMD_NODE_QUERY) || (cmd == CMD_NODE_DETECT);
}

uint32_t handle_command(const uint8_t *request, uint8_t *response,
                        size_t request_room, size_t response_room) {
  if (request[0] == CMD_NODE_DETECT) {
    return handle_detect_command(request, response, request_room);
  }
  return handle_query_command(request, response, request_room, response_room);
}

}  // namespace node_query
