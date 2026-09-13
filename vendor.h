/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * The vendor command block this probe implements, in the CMSIS-DAP
 * ID_DAP_Vendor0..ID_DAP_Vendor31 range.
 *
 * Every one of these answers with the command ID echoed back, then a status
 * byte, then whatever the command defines. Zero is success and anything else
 * is a failure, so one table covers the lot; 0xFF (DAP_ERROR) stays the
 * generic failure a command can fall back on.
 *
 * The commands themselves are documented where they are implemented:
 * dp_connect.h for the debug port bring-up, rtt.h for RTT, node_query.h for
 * the fused mux+connect+RTT round trip, rp2040.h for RP2040 flashing,
 * calibrate.h for SWD clock self-calibration.
 */

#ifndef VENDOR_H
#define VENDOR_H

#include <stdint.h>

namespace vendor {

constexpr uint8_t CMD_MUX_SELECT = 0x80U;   // SWD mux position, see swdmux.h
constexpr uint8_t CMD_RTT_START  = 0x81U;
constexpr uint8_t CMD_RTT_STOP   = 0x82U;
constexpr uint8_t CMD_RTT_STATUS = 0x83U;
constexpr uint8_t CMD_RTT_READ   = 0x84U;
constexpr uint8_t CMD_RTT_WRITE  = 0x85U;
constexpr uint8_t CMD_DP_CONNECT = 0x86U;
constexpr uint8_t CMD_NODE_QUERY = 0x87U;

// RP2040 bootrom flashing, see rp2040.h. Contiguous, and handled as a block.
constexpr uint8_t CMD_RP_ATTACH         = 0x88U;
constexpr uint8_t CMD_RP_CORE           = 0x89U;
constexpr uint8_t CMD_RP_CALL           = 0x8AU;
constexpr uint8_t CMD_RP_FLASH_PREP     = 0x8BU;
constexpr uint8_t CMD_RP_FLASH_ERASE    = 0x8CU;
constexpr uint8_t CMD_RP_FLASH_STAGE    = 0x8DU;
constexpr uint8_t CMD_RP_FLASH_PROGRAM  = 0x8EU;
constexpr uint8_t CMD_RP_FLASH_FINISH   = 0x8FU;
constexpr uint8_t CMD_RP_FLASH_CRC      = 0x90U;

// SWD clock self-calibration, see calibrate.h.
constexpr uint8_t CMD_CALIBRATE         = 0x91U;

// Detect-only: mux select + DP connect, nothing else. See node_query.h.
constexpr uint8_t CMD_NODE_DETECT       = 0x92U;

// RP2040 rescue mode, see dp_connect.h -- a debug port operation, not a
// bootrom one, which is why it lives with DP_Connect rather than in rp2040.h.
constexpr uint8_t CMD_RP_RESCUE         = 0x93U;

enum Status : uint8_t {
  STATUS_OK            = 0x00U,
  STATUS_NOT_STARTED   = 0x01U,  // no RTT_Start has succeeded
  STATUS_NOT_CONNECTED = 0x02U,  // debug port is not in SWD mode
  STATUS_TRANSFER      = 0x03U,  // a DP/AP transfer failed, or none answered
  STATUS_DAP_BUSY      = 0x04U,  // the DP had a sticky error on arrival
  STATUS_UNSUPPORTED   = 0x05U,  // the MEM-AP cannot do byte accesses
  STATUS_NOT_FOUND     = 0x06U,  // no control block at/in the given range
  STATUS_BAD_CHANNEL   = 0x07U,  // channel out of range
  STATUS_NO_BUFFER     = 0x08U,  // channel exists but the target left it unset
  STATUS_CORRUPT       = 0x09U,  // ring offsets outside the buffer
  STATUS_BAD_REQUEST   = 0x0AU,  // malformed, truncated or oversized request
  STATUS_NO_POWER      = 0x0BU,  // the target never acknowledged power-up
  STATUS_MUX_FAILED    = 0x0CU,  // swdmux::select_pos() refused or is disabled
  STATUS_NOT_HALTED    = 0x0DU,  // the core had to be halted and is not
  STATUS_TIMEOUT       = 0x0EU,  // the target did not get there in time
  STATUS_NO_BOOTROM    = 0x0FU,  // no RP2040 bootrom at the expected address
  STATUS_NOT_ATTACHED  = 0x10U,  // no RP_Attach has succeeded
  STATUS_CALL_FAILED   = 0x11U,  // a ROM call halted away from the trampoline
  STATUS_NOT_MAPPED    = 0x12U,  // gpio pins are not memory-mapped yet
  STATUS_RESCUE_FAILED = 0x13U,  // the rescue DP did not release the reset
};

}  // namespace vendor

#endif  // VENDOR_H
