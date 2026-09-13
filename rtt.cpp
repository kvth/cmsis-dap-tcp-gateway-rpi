/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * SEGGER RTT vendor commands. See rtt.h for the wire format; this file is the
 * ring buffer protocol and the control block layout it runs on.
 */

#include "rtt.h"

#include <string.h>

#include "logging.h"
#include "target_mem.h"
#include "vendor.h"

namespace rtt {

using namespace vendor;

namespace {

// --- SEGGER RTT control block layout ---------------------------------------
//
//   struct SEGGER_RTT_CB {
//     char     acID[16];              // "SEGGER RTT" + NUL padding
//     int      MaxNumUpBuffers;
//     int      MaxNumDownBuffers;
//     RTT_BUFFER aUp[MaxNumUpBuffers];
//     RTT_BUFFER aDown[MaxNumDownBuffers];
//   };
//
//   struct RTT_BUFFER {
//     const char *sName;
//     char       *pBuffer;
//     unsigned    SizeOfBuffer;
//     unsigned    WrOff;              // up: written by target, down: by host
//     unsigned    RdOff;              // up: written by host,   down: by target
//     unsigned    Flags;
//   };

constexpr char     CB_ID[]        = "SEGGER RTT";
constexpr size_t   CB_ID_LEN      = sizeof(CB_ID);      // includes the NUL
constexpr uint32_t CB_OFF_MAX_UP   = 16U;
constexpr uint32_t CB_OFF_MAX_DOWN = 20U;
constexpr uint32_t CB_OFF_BUFFERS  = 24U;

constexpr uint32_t BUF_DESC_SIZE = 24U;
constexpr uint32_t BUF_OFF_NAME  = 0U;
constexpr uint32_t BUF_OFF_ADDR  = 4U;
constexpr uint32_t BUF_OFF_SIZE  = 8U;
constexpr uint32_t BUF_OFF_WROFF = 12U;
constexpr uint32_t BUF_OFF_RDOFF = 16U;
constexpr uint32_t BUF_OFF_FLAGS = 20U;

// A control block claiming more buffers than this is not one we found, it is
// whatever else happened to spell out the ID string. SEGGER's own default is
// three of each; the ceiling only has to be comfortably above real configs.
constexpr uint32_t MAX_BUFFERS = 64U;

// How much of the search range to pull over the wire at a time. Chunks
// overlap by CB_ID_LEN-1 so an ID straddling a boundary is still found.
constexpr size_t SEARCH_CHUNK = 512;

// --- located control block --------------------------------------------------

struct Session {
  bool     active;
  uint8_t  ap;
  uint32_t cb_addr;
  uint32_t num_up;
  uint32_t num_down;
};

Session session;

// --- little endian helpers --------------------------------------------------

uint16_t load_le16(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint32_t load_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
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

Status from_mem(target_mem::Status s) {
  switch (s) {
    case target_mem::Status::ok:             return STATUS_OK;
    case target_mem::Status::not_connected:  return STATUS_NOT_CONNECTED;
    case target_mem::Status::sticky_error:   return STATUS_DAP_BUSY;
    case target_mem::Status::transfer_fault: return STATUS_TRANSFER;
    case target_mem::Status::unsupported:    return STATUS_UNSUPPORTED;
  }
  return STATUS_TRANSFER;
}

// --- control block discovery ------------------------------------------------

// Read the buffer counts and check they are plausible. This is the second half
// of recognising a control block: the ID string alone can appear in a string
// table or in a stale copy of the struct.
Status read_header(target_mem::Burst &bus, uint32_t cb_addr,
                   uint32_t *num_up, uint32_t *num_down) {
  uint8_t header[8];
  const target_mem::Status st =
      bus.read(cb_addr + CB_OFF_MAX_UP, header, sizeof(header));
  if (st != target_mem::Status::ok) {
    return from_mem(st);
  }

  const uint32_t up   = load_le32(header + 0);
  const uint32_t down = load_le32(header + 4);
  if ((up > MAX_BUFFERS) || (down > MAX_BUFFERS) || ((up + down) == 0U)) {
    return STATUS_NOT_FOUND;
  }

  *num_up   = up;
  *num_down = down;
  return STATUS_OK;
}

Status verify_at(target_mem::Burst &bus, uint32_t cb_addr,
                 uint32_t *num_up, uint32_t *num_down) {
  uint8_t id[CB_ID_LEN];
  const target_mem::Status st = bus.read(cb_addr, id, sizeof(id));
  if (st != target_mem::Status::ok) {
    return from_mem(st);
  }
  if (memcmp(id, CB_ID, CB_ID_LEN) != 0) {
    return STATUS_NOT_FOUND;
  }
  return read_header(bus, cb_addr, num_up, num_down);
}

// Scan [start, start+len) for the ID string. The target writes the ID last
// when it initialises the block, so finding it means the rest is already
// there.
Status search(target_mem::Burst &bus, uint32_t start, uint32_t len,
              uint32_t *cb_addr, uint32_t *num_up, uint32_t *num_down) {
  if (len < CB_ID_LEN) {
    return STATUS_NOT_FOUND;
  }

  uint8_t      buf[SEARCH_CHUNK];
  const size_t overlap = CB_ID_LEN - 1U;
  const size_t stride  = SEARCH_CHUNK - overlap;

  for (uint32_t off = 0U; off + CB_ID_LEN <= len; off += (uint32_t)stride) {
    size_t chunk = len - off;
    if (chunk > SEARCH_CHUNK) {
      chunk = SEARCH_CHUNK;
    }

    if (bus.read(start + off, buf, chunk) != target_mem::Status::ok) {
      // Part of the range is not memory the AP can reach. That is expected
      // when the caller passes a whole RAM window, so shake off the sticky
      // error and carry on past it rather than giving up on the search.
      const target_mem::Status cleared = bus.clear_sticky();
      if (cleared != target_mem::Status::ok) {
        return from_mem(cleared);
      }
      continue;
    }

    for (size_t i = 0; i + CB_ID_LEN <= chunk; i++) {
      if (memcmp(buf + i, CB_ID, CB_ID_LEN) != 0) {
        continue;
      }
      const uint32_t candidate = start + off + (uint32_t)i;
      if (read_header(bus, candidate, num_up, num_down) == STATUS_OK) {
        *cb_addr = candidate;
        return STATUS_OK;
      }
    }
  }

  return STATUS_NOT_FOUND;
}

// --- ring buffer access ------------------------------------------------------

// One channel's descriptor, as read from the target.
struct Descriptor {
  uint32_t desc_addr;
  uint32_t name_addr;
  uint32_t buffer;
  uint32_t size;
  uint32_t write_off;
  uint32_t read_off;
  uint32_t flags;
};

// `direction` is 0 for an up buffer (target to host), 1 for a down buffer.
Status read_descriptor(target_mem::Burst &bus, unsigned direction,
                       unsigned channel, Descriptor *out) {
  const uint32_t limit = (direction == 0U) ? session.num_up : session.num_down;
  if (channel >= limit) {
    return STATUS_BAD_CHANNEL;
  }

  const uint32_t index =
      (direction == 0U) ? channel : (session.num_up + channel);
  out->desc_addr = session.cb_addr + CB_OFF_BUFFERS + index * BUF_DESC_SIZE;

  uint8_t desc[BUF_DESC_SIZE];
  const target_mem::Status st = bus.read(out->desc_addr, desc, sizeof(desc));
  if (st != target_mem::Status::ok) {
    return from_mem(st);
  }

  out->name_addr = load_le32(desc + BUF_OFF_NAME);
  out->buffer    = load_le32(desc + BUF_OFF_ADDR);
  out->size      = load_le32(desc + BUF_OFF_SIZE);
  out->write_off = load_le32(desc + BUF_OFF_WROFF);
  out->read_off  = load_le32(desc + BUF_OFF_RDOFF);
  out->flags     = load_le32(desc + BUF_OFF_FLAGS);
  return STATUS_OK;
}

// A channel the target declared but never gave storage to is a normal state
// (SEGGER's default control block reserves spare slots), so it gets its own
// status rather than being reported as corruption.
Status check_usable(const Descriptor &d) {
  if ((d.buffer == 0U) || (d.size == 0U)) {
    return STATUS_NO_BUFFER;
  }
  if ((d.write_off >= d.size) || (d.read_off >= d.size)) {
    return STATUS_CORRUPT;
  }
  return STATUS_OK;
}

// --- command handlers ---------------------------------------------------------

// Every response is the command ID, the status, and then whatever the command
// defines. A failure still emits the full fixed part so the client can parse
// the response without having to know which fields a failure omits.
uint32_t reply(uint8_t *response, uint8_t cmd, Status status,
               size_t body_len, uint32_t request_len) {
  response[0] = cmd;
  response[1] = (uint8_t)status;
  if (status != STATUS_OK) {
    memset(response + 2, 0, body_len);
  }
  return (request_len << 16) | (uint32_t)(2U + body_len);
}

uint32_t cmd_start(const uint8_t *request, uint8_t *response) {
  constexpr size_t REQUEST_LEN = 1U + 9U;   // id, ap, address, search_len
  constexpr size_t BODY_LEN    = 6U;        // cb_addr, num_up, num_down

  const uint8_t  ap         = request[1];
  const uint32_t address    = load_le32(request + 2);
  const uint32_t search_len = load_le32(request + 6);

  session.active = false;

  target_mem::Burst bus(ap);
  Status status = from_mem(bus.status());

  uint32_t cb_addr  = address;
  uint32_t num_up   = 0U;
  uint32_t num_down = 0U;

  if (status == STATUS_OK) {
    status = (search_len == 0U)
                 ? verify_at(bus, address, &num_up, &num_down)
                 : search(bus, address, search_len, &cb_addr, &num_up, &num_down);
  }

  if (status == STATUS_OK) {
    session.active   = true;
    session.ap       = ap;
    session.cb_addr  = cb_addr;
    session.num_up   = num_up;
    session.num_down = num_down;
    LOGI_KV("rtt control block located",
            "ap=%u cb_addr=0x%08x up=%u down=%u",
            (unsigned)ap, (unsigned)cb_addr, (unsigned)num_up,
            (unsigned)num_down);
  } else {
    LOGD_KV("rtt start failed",
            "ap=%u address=0x%08x search_len=%u status=%u",
            (unsigned)ap, (unsigned)address, (unsigned)search_len,
            (unsigned)status);
  }

  uint8_t *body = response + 2;
  body = store_le32(body, cb_addr);
  *body++ = (uint8_t)num_up;
  *body++ = (uint8_t)num_down;
  return reply(response, CMD_RTT_START, status, BODY_LEN, REQUEST_LEN);
}

uint32_t cmd_stop(uint8_t *response) {
  session.active = false;
  return reply(response, CMD_RTT_STOP, STATUS_OK, 0U, 1U);
}

uint32_t cmd_status(const uint8_t *request, uint8_t *response) {
  constexpr size_t REQUEST_LEN = 1U + 2U;   // id, direction, channel
  constexpr size_t FIXED_LEN   = 21U;       // 5 words plus the name length

  const unsigned direction = request[1];
  const unsigned channel   = request[2];

  uint8_t *body = response + 2;
  memset(body, 0, FIXED_LEN);

  if (!session.active) {
    return reply(response, CMD_RTT_STATUS, STATUS_NOT_STARTED, FIXED_LEN,
                 REQUEST_LEN);
  }
  if (direction > 1U) {
    return reply(response, CMD_RTT_STATUS, STATUS_BAD_REQUEST, FIXED_LEN,
                 REQUEST_LEN);
  }

  target_mem::Burst bus(session.ap);
  Status     status = from_mem(bus.status());
  Descriptor desc   = {};
  char       name[NAME_MAX];
  size_t     name_len = 0U;

  if (status == STATUS_OK) {
    status = read_descriptor(bus, direction, channel, &desc);
  }
  if ((status == STATUS_OK) && (desc.name_addr != 0U)) {
    // A name that will not read is not worth failing the command over: the
    // offsets are what the caller is here for.
    if (bus.read(desc.name_addr, name, sizeof(name)) ==
        target_mem::Status::ok) {
      while ((name_len < sizeof(name)) && (name[name_len] != '\0')) {
        name_len++;
      }
    } else {
      bus.clear_sticky();
    }
  }

  body = store_le32(body, desc.buffer);
  body = store_le32(body, desc.size);
  body = store_le32(body, desc.write_off);
  body = store_le32(body, desc.read_off);
  body = store_le32(body, desc.flags);
  *body++ = (uint8_t)name_len;
  if (status == STATUS_OK) {
    memcpy(body, name, name_len);
  } else {
    name_len = 0U;
  }

  return reply(response, CMD_RTT_STATUS, status, FIXED_LEN + name_len,
               REQUEST_LEN);
}

uint32_t cmd_read(const uint8_t *request, uint8_t *response,
                  size_t response_room) {
  constexpr size_t REQUEST_LEN = 1U + 3U;   // id, channel, max_len
  constexpr size_t HEADER_LEN  = 4U;        // id, status, length

  const unsigned channel = request[1];
  size_t         want    = load_le16(request + 2);

  // Never promise more than fits in what is left of the response packet: a
  // batched request that overran it would produce a response the client drops
  // whole, losing the data we already took out of the ring.
  const size_t room = (response_room > HEADER_LEN)
                          ? (response_room - HEADER_LEN)
                          : 0U;
  if (want > room) {
    want = room;
  }

  uint8_t *body   = response + 2;
  uint8_t *data   = response + HEADER_LEN;
  size_t   copied = 0U;

  if (!session.active) {
    store_le16(body, 0U);
    return reply(response, CMD_RTT_READ, STATUS_NOT_STARTED, 2U, REQUEST_LEN);
  }

  target_mem::Burst bus(session.ap);
  Status     status = from_mem(bus.status());
  Descriptor desc   = {};

  if (status == STATUS_OK) {
    status = read_descriptor(bus, 0U, channel, &desc);
  }
  if (status == STATUS_OK) {
    status = check_usable(desc);
  }

  if (status == STATUS_OK) {
    uint32_t read_off = desc.read_off;

    // Up to two runs: from the read offset to whichever comes first of the
    // write offset and the end of the ring, then from the start of the ring.
    while ((copied < want) && (read_off != desc.write_off)) {
      const uint32_t run_end =
          (desc.write_off > read_off) ? desc.write_off : desc.size;
      size_t run = run_end - read_off;
      if (run > (want - copied)) {
        run = want - copied;
      }

      const target_mem::Status st =
          bus.read(desc.buffer + read_off, data + copied, run);
      if (st != target_mem::Status::ok) {
        status = from_mem(st);
        break;
      }

      copied  += run;
      read_off = (uint32_t)(read_off + run);
      if (read_off == desc.size) {
        read_off = 0U;
      }
    }

    // Publish the new read offset only once the data is safely in hand, so a
    // failed transfer leaves the ring where it was and the bytes are still
    // there to be read again.
    if ((status == STATUS_OK) && (copied != 0U)) {
      const target_mem::Status st =
          bus.write_u32(desc.desc_addr + BUF_OFF_RDOFF, read_off);
      if (st != target_mem::Status::ok) {
        status = from_mem(st);
      }
    }
  }

  if (status != STATUS_OK) {
    copied = 0U;
  }
  store_le16(body, (uint16_t)copied);
  response[0] = CMD_RTT_READ;
  response[1] = (uint8_t)status;
  return ((uint32_t)REQUEST_LEN << 16) | (uint32_t)(HEADER_LEN + copied);
}

uint32_t cmd_write(const uint8_t *request, uint8_t *response,
                   size_t request_room) {
  constexpr size_t BODY_LEN = 2U;           // accepted

  const unsigned channel = request[1];
  const size_t   len     = load_le16(request + 2);
  const uint8_t *data    = request + 4;
  const uint32_t request_len = (uint32_t)(4U + len);

  uint8_t *body = response + 2;

  // The payload has to be inside the packet it arrived in. A length that runs
  // past the end is a malformed request, not a short read of whatever memory
  // happens to follow the receive buffer.
  if ((4U + len) > request_room) {
    store_le16(body, 0U);
    return reply(response, CMD_RTT_WRITE, STATUS_BAD_REQUEST, BODY_LEN,
                 (uint32_t)request_room);
  }

  if (!session.active) {
    store_le16(body, 0U);
    return reply(response, CMD_RTT_WRITE, STATUS_NOT_STARTED, BODY_LEN,
                 request_len);
  }

  target_mem::Burst bus(session.ap);
  Status     status  = from_mem(bus.status());
  Descriptor desc    = {};
  size_t     written = 0U;

  if (status == STATUS_OK) {
    status = read_descriptor(bus, 1U, channel, &desc);
  }
  if (status == STATUS_OK) {
    status = check_usable(desc);
  }

  if (status == STATUS_OK) {
    uint32_t write_off = desc.write_off;

    // One slot is always left empty: a full ring and an empty one would
    // otherwise both have the two offsets equal.
    size_t free_bytes = (desc.read_off > write_off)
                            ? (desc.read_off - write_off - 1U)
                            : (desc.size - write_off + desc.read_off - 1U);
    size_t want = (len < free_bytes) ? len : free_bytes;

    while (written < want) {
      const uint32_t run_end =
          (desc.read_off > write_off) ? (desc.read_off - 1U) : desc.size;
      size_t run = run_end - write_off;
      if (run > (want - written)) {
        run = want - written;
      }
      if (run == 0U) {
        break;
      }

      const target_mem::Status st =
          bus.write(desc.buffer + write_off, data + written, run);
      if (st != target_mem::Status::ok) {
        status = from_mem(st);
        break;
      }

      written  += run;
      write_off = (uint32_t)(write_off + run);
      if (write_off == desc.size) {
        write_off = 0U;
      }
    }

    // As with the read: the offset moves only after the bytes are in place,
    // or the target would read a window that is not filled in yet.
    if ((status == STATUS_OK) && (written != 0U)) {
      const target_mem::Status st =
          bus.write_u32(desc.desc_addr + BUF_OFF_WROFF, write_off);
      if (st != target_mem::Status::ok) {
        status = from_mem(st);
      }
    }
  }

  if (status != STATUS_OK) {
    written = 0U;
  }
  store_le16(body, (uint16_t)written);
  response[0] = CMD_RTT_WRITE;
  response[1] = (uint8_t)status;
  return (request_len << 16) | (uint32_t)(2U + BODY_LEN);
}

}  // namespace

bool handles(uint8_t cmd) {
  return (cmd >= CMD_RTT_START) && (cmd <= CMD_RTT_WRITE);
}

void reset() {
  session.active = false;
}

uint32_t handle_command(const uint8_t *request, uint8_t *response,
                        size_t request_room, size_t response_room) {
  const uint8_t cmd = request[0];

  // Bytes each command's fixed part occupies, command ID included. Reading a
  // field that is not in the packet would be reading whatever follows it.
  size_t need = 1U;
  switch (cmd) {
    case CMD_RTT_START:  need = 10U; break;   // ap, address, search_len
    case CMD_RTT_STOP:   need =  1U; break;
    case CMD_RTT_STATUS: need =  3U; break;   // direction, channel
    case CMD_RTT_READ:   need =  4U; break;   // channel, max_len
    case CMD_RTT_WRITE:  need =  4U; break;   // channel, len (payload checked below)
    default:
      response[0] = 0xFFU;    // ID_DAP_Invalid
      return (1U << 16) | 1U;
  }

  if (request_room < need) {
    // Truncated. Claim what is left of the packet so a batch stops here
    // instead of carrying on from an offset past its end.
    return reply(response, cmd, STATUS_BAD_REQUEST, 0U, (uint32_t)request_room);
  }

  switch (cmd) {
    case CMD_RTT_START:  return cmd_start(request, response);
    case CMD_RTT_STOP:   return cmd_stop(response);
    case CMD_RTT_STATUS: return cmd_status(request, response);
    case CMD_RTT_READ:   return cmd_read(request, response, response_room);
    default:         return cmd_write(request, response, request_room);
  }
}

}  // namespace rtt
