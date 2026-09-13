/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Debug port bring-up. See dp_connect.h for the wire format and for what this
 * does to a session already in progress.
 */

#include "dp_connect.h"

#include <string.h>

#include "logging.h"
#include "rp2040.h"
#include "rtt.h"
#include "swd_port.h"
#include "vendor.h"

namespace dp {

using namespace vendor;

namespace {

// Switch sequences, taken from OpenOCD's src/jtag/swd.h. Each byte goes out
// low bit first, and the bit counts are what that file records: they are not
// always the byte count times eight, so keep the two together.
const uint8_t SEQ_LINE_RESET[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
};
constexpr uint32_t SEQ_LINE_RESET_BITS = 64U;

const uint8_t SEQ_JTAG_TO_SWD[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,       // line reset
    0x9E, 0xE7,                                     // JTAG-to-SWD select
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,       // line reset
    0x00,                                           // idle
};
constexpr uint32_t SEQ_JTAG_TO_SWD_BITS = 136U;

const uint8_t SEQ_JTAG_TO_DORMANT[] = {
    0xFF, 0x75, 0x77, 0x77, 0x67,
};
constexpr uint32_t SEQ_JTAG_TO_DORMANT_BITS = 40U;

const uint8_t SEQ_DORMANT_TO_SWD[] = {
    0xFF,                                           // 8 clocks high
    0x92, 0xF3, 0x09, 0x62, 0x95, 0x2D, 0x85, 0x86, // selection alert
    0xE9, 0xAF, 0xDD, 0xE3, 0xA2, 0x0E, 0xBC, 0x19,
    0xA0, 0xF1, 0xFF,                               // SWD activation code
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,       // line reset
    0x00,                                           // idle
};
constexpr uint32_t SEQ_DORMANT_TO_SWD_BITS = 224U;

static_assert(sizeof(SEQ_LINE_RESET) * 8U >= SEQ_LINE_RESET_BITS &&
              sizeof(SEQ_JTAG_TO_SWD) * 8U >= SEQ_JTAG_TO_SWD_BITS &&
              sizeof(SEQ_JTAG_TO_DORMANT) * 8U >= SEQ_JTAG_TO_DORMANT_BITS &&
              sizeof(SEQ_DORMANT_TO_SWD) * 8U >= SEQ_DORMANT_TO_SWD_BITS,
              "a switch sequence claims more bits than it has bytes for");

// DP CTRL/STAT power handshake.
constexpr uint32_t CTRL_CDBGPWRUPREQ  = 1U << 28;
constexpr uint32_t CTRL_CDBGPWRUPACK  = 1U << 29;
constexpr uint32_t CTRL_CSYSPWRUPREQ  = 1U << 30;
constexpr uint32_t CTRL_CSYSPWRUPACK  = 1U << 31;
constexpr uint32_t CTRL_PWRUP_REQ = CTRL_CDBGPWRUPREQ | CTRL_CSYSPWRUPREQ;
constexpr uint32_t CTRL_PWRUP_ACK = CTRL_CDBGPWRUPACK | CTRL_CSYSPWRUPACK;

// Every sticky flag DP ABORT can clear.
constexpr uint32_t ABORT_CLEAR_ALL =
    swd::ABORT_STKCMPCLR | swd::ABORT_STKERRCLR |
    swd::ABORT_WDERRCLR | swd::ABORT_ORUNERRCLR;

// How many times to re-read CTRL/STAT waiting for the power-up acknowledge.
constexpr unsigned POWER_UP_POLLS = 100U;

// The RP2040's rescue debug port: the same part number as its cores, with
// instance 0xF instead of 0 or 1. See rescue() below for what it does.
constexpr uint32_t RESCUE_TARGETSEL = 0xF1002927U;

// The packet request byte for a DP write to TARGETSEL (register 0x0C): start
// bit, APnDP and RnW clear, A[3:2] = 0b11, even parity over those four, stop,
// park.
constexpr uint8_t TARGETSEL_REQUEST = 0x99U;

uint32_t load_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint8_t *store_le32(uint8_t *p, uint32_t v) {
  *p++ = (uint8_t)v;
  *p++ = (uint8_t)(v >> 8);
  *p++ = (uint8_t)(v >> 16);
  *p++ = (uint8_t)(v >> 24);
  return p;
}

Status dp_read(uint32_t reg, uint32_t *out) {
  uint32_t value = 0U;
  const uint8_t ack =
      swd::transfer(swd::REQ_RnW | (reg & swd::REQ_ADDR), &value);
  if (ack != swd::ACK_OK) {
    return STATUS_TRANSFER;
  }
  if (out != nullptr) {
    *out = value;
  }
  return STATUS_OK;
}

Status dp_write(uint32_t reg, uint32_t value) {
  uint32_t v = value;
  const uint8_t ack = swd::transfer(reg & swd::REQ_ADDR, &v);
  return (ack == swd::ACK_OK) ? STATUS_OK : STATUS_TRANSFER;
}

// Write DP TARGETSEL, which nothing acknowledges.
//
// After a line reset every DP on a multi-drop wire is deselected, so none of
// them drives the response to this write -- which a normal transfer would call
// a protocol error. The three phases go out as raw sequences instead: the
// packet request, the three ack bits nobody answers with, and the data.
void write_targetsel(uint32_t targetsel) {
  uint8_t data[5];
  store_le32(data, targetsel);
  data[4] = (uint8_t)(__builtin_popcount(targetsel) & 1U);   // parity

  const uint8_t request = TARGETSEL_REQUEST;
  swd::sequence_swd(8U, &request, nullptr);

  uint8_t discarded = 0U;
  swd::sequence_swd(swd::SEQ_CAPTURE | 5U, nullptr, &discarded);

  swd::sequence_swd(32U + 1U, data, nullptr);
}

// One bring-up attempt: switch the wire, optionally pick a DP, and see
// whether anything identifies itself.
Status attempt(bool through_dormant, bool multidrop, uint32_t targetsel,
               uint32_t *dpidr) {
  if (through_dormant) {
    swd::sequence_swj(SEQ_LINE_RESET_BITS, SEQ_LINE_RESET);
    swd::sequence_swj(SEQ_JTAG_TO_DORMANT_BITS, SEQ_JTAG_TO_DORMANT);
    swd::sequence_swj(SEQ_DORMANT_TO_SWD_BITS, SEQ_DORMANT_TO_SWD);
  } else {
    swd::sequence_swj(SEQ_JTAG_TO_SWD_BITS, SEQ_JTAG_TO_SWD);
  }

  if (multidrop) {
    swd::sequence_swj(SEQ_LINE_RESET_BITS, SEQ_LINE_RESET);
    write_targetsel(targetsel);
  }

  // DPIDR is the only register readable before the sticky bits are cleared,
  // and reading it is what confirms a DP is listening at all.
  const Status st = dp_read(swd::DP_REG_DPIDR, dpidr);
  if (st != STATUS_OK) {
    return st;
  }
  // A wire with nothing on it floats to one or the other of these.
  if ((*dpidr == 0x00000000U) || (*dpidr == 0xFFFFFFFFU)) {
    return STATUS_TRANSFER;
  }
  return STATUS_OK;
}

Status power_up(uint32_t *ctrl_stat) {
  Status st = dp_write(swd::DP_REG_ABORT, ABORT_CLEAR_ALL);
  if (st != STATUS_OK) {
    return st;
  }
  st = dp_write(swd::DP_REG_SELECT, 0x00000000U);
  if (st != STATUS_OK) {
    return st;
  }
  st = dp_write(swd::DP_REG_CTRL_STAT, CTRL_PWRUP_REQ);
  if (st != STATUS_OK) {
    return st;
  }

  // Nothing reaches memory until the target grants debug and system power.
  for (unsigned i = 0; i < POWER_UP_POLLS; i++) {
    st = dp_read(swd::DP_REG_CTRL_STAT, ctrl_stat);
    if (st != STATUS_OK) {
      return st;
    }
    if ((*ctrl_stat & CTRL_PWRUP_ACK) == CTRL_PWRUP_ACK) {
      return STATUS_OK;
    }
  }
  return STATUS_NO_POWER;
}

// The rescue sequence, as OpenOCD's target/rp2040.cfg runs it under RESCUE.
//
// Instance 0xF on the wire is not a core's debug port at all: its
// CDBGPWRUPREQ is wired to the RP2040's power-on state machine, and asserting
// it resets the chip with a flag set in VREG_AND_POR_CHIP_RESET. Clearing it
// again releases the reset, and the bootrom sees that flag on the way up and
// stops before it touches flash.
//
// Nothing acknowledges the power-up request here -- the PSM is the thing being
// held in reset -- so unlike power_up() this must not wait for one. That is
// OpenOCD's `-ignore-syspwrupack` on the rescue DAP.
Status rescue(uint32_t targetsel, unsigned switch_mode, uint32_t *dpidr,
              uint32_t *ctrl_stat) {
  const bool try_direct  = (switch_mode == 0U) || (switch_mode == 1U);
  const bool try_dormant = (switch_mode == 0U) || (switch_mode == 2U);

  Status st = STATUS_TRANSFER;
  if (try_direct) {
    st = attempt(false, true, targetsel, dpidr);
  }
  if ((st != STATUS_OK) && try_dormant) {
    st = attempt(true, true, targetsel, dpidr);
  }
  if (st != STATUS_OK) {
    return st;
  }

  st = dp_write(swd::DP_REG_ABORT, ABORT_CLEAR_ALL);
  if (st != STATUS_OK) {
    return st;
  }
  st = dp_write(swd::DP_REG_SELECT, 0x00000000U);
  if (st != STATUS_OK) {
    return st;
  }

  // Assert: the chip goes into reset here and stays there.
  st = dp_write(swd::DP_REG_CTRL_STAT, CTRL_PWRUP_REQ);
  if (st != STATUS_OK) {
    return st;
  }
  // Release: the bootrom runs from here and halts itself.
  st = dp_write(swd::DP_REG_CTRL_STAT, 0x00000000U);
  if (st != STATUS_OK) {
    return st;
  }

  st = dp_read(swd::DP_REG_CTRL_STAT, ctrl_stat);
  if (st != STATUS_OK) {
    return st;
  }
  // The same check OpenOCD prints "Rescue failed" on: every power request and
  // acknowledge bit has to have gone away, or the reset is still asserted.
  if ((*ctrl_stat & (CTRL_PWRUP_REQ | CTRL_PWRUP_ACK)) != 0U) {
    return STATUS_RESCUE_FAILED;
  }
  return STATUS_OK;
}

uint32_t cmd_rescue(const uint8_t *request, uint8_t *response,
                    size_t request_room) {
  constexpr size_t REQUEST_LEN = 1U + 1U + 4U;   // id, flags, targetsel
  constexpr size_t BODY_LEN    = 8U;             // dpidr, ctrl_stat

  response[0] = CMD_RP_RESCUE;

  if (request_room < REQUEST_LEN) {
    response[1] = STATUS_BAD_REQUEST;
    memset(response + 2, 0, BODY_LEN);
    return ((uint32_t)request_room << 16) | (uint32_t)(2U + BODY_LEN);
  }

  const uint8_t  flags       = request[1];
  const unsigned switch_mode = (flags >> 1) & 0x03U;
  uint32_t       targetsel   = load_le32(request + 2);
  if (targetsel == 0U) {
    targetsel = RESCUE_TARGETSEL;
  }

  uint32_t dpidr     = 0U;
  uint32_t ctrl_stat = 0U;
  Status   status    = STATUS_OK;

  if (!swd::port_is_swd()) {
    status = STATUS_NOT_CONNECTED;
  } else if (switch_mode > 2U) {
    status = STATUS_BAD_REQUEST;
  } else {
    // This resets the chip, so everything anything here knew about the target
    // is gone -- including, unlike DP_Connect, the target's own RAM.
    swd::shadow_reset();
    rtt::reset();
    rp2040::reset();

    status = rescue(targetsel, switch_mode, &dpidr, &ctrl_stat);
  }

  if (status == STATUS_OK) {
    LOGI_KV("rp2040 rescued",
            "dpidr=0x%08x ctrl_stat=0x%08x targetsel=0x%08x",
            (unsigned)dpidr, (unsigned)ctrl_stat, (unsigned)targetsel);
  } else {
    LOGW_KV("rp2040 rescue failed",
            "status=%u dpidr=0x%08x ctrl_stat=0x%08x targetsel=0x%08x",
            (unsigned)status, (unsigned)dpidr, (unsigned)ctrl_stat,
            (unsigned)targetsel);
    dpidr     = 0U;
    ctrl_stat = 0U;
  }

  response[1] = (uint8_t)status;
  uint8_t *body = response + 2;
  body = store_le32(body, dpidr);
  body = store_le32(body, ctrl_stat);
  return ((uint32_t)REQUEST_LEN << 16) | (uint32_t)(2U + BODY_LEN);
}

}  // namespace

bool handles(uint8_t cmd) {
  return (cmd == CMD_DP_CONNECT) || (cmd == CMD_RP_RESCUE);
}

uint32_t handle_command(const uint8_t *request, uint8_t *response,
                        size_t request_room) {
  constexpr size_t REQUEST_LEN = 1U + 1U + 4U;   // id, flags, targetsel
  constexpr size_t BODY_LEN    = 8U;             // dpidr, ctrl_stat

  if (request[0] == CMD_RP_RESCUE) {
    return cmd_rescue(request, response, request_room);
  }

  response[0] = CMD_DP_CONNECT;

  if (request_room < REQUEST_LEN) {
    response[1] = STATUS_BAD_REQUEST;
    memset(response + 2, 0, BODY_LEN);
    return ((uint32_t)request_room << 16) | (uint32_t)(2U + BODY_LEN);
  }

  const uint8_t  flags     = request[1];
  const uint32_t targetsel = load_le32(request + 2);
  const bool     multidrop = (flags & 0x01U) != 0U;
  const unsigned switch_mode = (flags >> 1) & 0x03U;

  uint32_t dpidr     = 0U;
  uint32_t ctrl_stat = 0U;
  Status   status    = STATUS_OK;

  if (!swd::port_is_swd()) {
    status = STATUS_NOT_CONNECTED;
  } else if (switch_mode > 2U) {
    status = STATUS_BAD_REQUEST;
  } else {
    // The link is about to be torn down and rebuilt, so nothing either side
    // remembers about the old one survives. Drop ours before touching the
    // wire, not after, so a failure part way through cannot leave a stale
    // shadow behind for the RTT commands to restore from.
    swd::shadow_reset();
    rtt::reset();
    rp2040::reset();

    // Which switch sequence a part wants is not something it can be asked, so
    // unless told, try the direct one and then the route through the dormant
    // state -- the same order OpenOCD uses.
    const bool try_direct  = (switch_mode == 0U) || (switch_mode == 1U);
    const bool try_dormant = (switch_mode == 0U) || (switch_mode == 2U);

    status = STATUS_TRANSFER;
    if (try_direct) {
      status = attempt(false, multidrop, targetsel, &dpidr);
    }
    if ((status != STATUS_OK) && try_dormant) {
      status = attempt(true, multidrop, targetsel, &dpidr);
    }

    if (status == STATUS_OK) {
      status = power_up(&ctrl_stat);
    }
  }

  if (status == STATUS_OK) {
    LOGI_KV("debug port connected",
            "dpidr=0x%08x ctrl_stat=0x%08x multidrop=%s targetsel=0x%08x",
            (unsigned)dpidr, (unsigned)ctrl_stat, multidrop ? "yes" : "no",
            (unsigned)(multidrop ? targetsel : 0U));
  } else {
    dpidr     = 0U;
    ctrl_stat = 0U;
    LOGW_KV("debug port bring-up failed",
            "status=%u multidrop=%s targetsel=0x%08x",
            (unsigned)status, multidrop ? "yes" : "no", (unsigned)targetsel);
  }

  response[1] = (uint8_t)status;
  uint8_t *body = response + 2;
  body = store_le32(body, dpidr);
  body = store_le32(body, ctrl_stat);

  return ((uint32_t)REQUEST_LEN << 16) | (uint32_t)(2U + BODY_LEN);
}

}  // namespace dp
