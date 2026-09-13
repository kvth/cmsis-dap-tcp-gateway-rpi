/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * The raw SWD transfer primitive, plus a shadow of the DP/AP registers the
 * client believes it owns.
 *
 * A probe-side command such as the RTT vendor commands (see rtt.h) has to run
 * its own DP and AP transfers on a wire the client is also driving. The client
 * caches DP SELECT and the MEM-AP's CSW and TAR and only rewrites them when it
 * thinks they changed, so a transfer we issue behind its back would leave the
 * hardware disagreeing with that cache and corrupt the client's next access.
 * Tracking every write the client makes lets us put those registers back
 * before returning, which is what makes probe-side memory access safe to mix
 * with a live OpenOCD session.
 *
 * Implemented in DAP.cpp, which owns the bitbang. Only the DAP command
 * handlers and target_mem.cpp should need this.
 */

#ifndef SWD_PORT_H
#define SWD_PORT_H

#include <stdint.h>

namespace swd {

// SWD DP register addresses, as the A[3:2] field of a transfer request.
constexpr uint32_t DP_REG_DPIDR     = 0x00U;   // read side of address 0
constexpr uint32_t DP_REG_ABORT     = 0x00U;   // write side of address 0
constexpr uint32_t DP_REG_CTRL_STAT = 0x04U;
constexpr uint32_t DP_REG_SELECT    = 0x08U;   // write only
constexpr uint32_t DP_REG_RDBUFF    = 0x0CU;   // read only

// MEM-AP register addresses within AP bank 0.
constexpr uint32_t AP_REG_CSW = 0x00U;
constexpr uint32_t AP_REG_TAR = 0x04U;
constexpr uint32_t AP_REG_DRW = 0x0CU;

// DP CTRL/STAT sticky flags, and the DP ABORT bits that clear them.
constexpr uint32_t CTRL_STICKYORUN = 1U << 1;
constexpr uint32_t CTRL_STICKYCMP  = 1U << 4;
constexpr uint32_t CTRL_STICKYERR  = 1U << 5;
constexpr uint32_t CTRL_WDATAERR   = 1U << 7;
constexpr uint32_t CTRL_STICKY_ANY =
    CTRL_STICKYORUN | CTRL_STICKYCMP | CTRL_STICKYERR | CTRL_WDATAERR;

constexpr uint32_t ABORT_STKCMPCLR  = 1U << 1;
constexpr uint32_t ABORT_STKERRCLR  = 1U << 2;
constexpr uint32_t ABORT_WDERRCLR   = 1U << 3;
constexpr uint32_t ABORT_ORUNERRCLR = 1U << 4;

// MEM-AP CSW fields we drive; everything else is left as the client set it.
constexpr uint32_t CSW_SIZE_MASK    = 0x7U;
constexpr uint32_t CSW_SIZE_BYTE    = 0x0U;
constexpr uint32_t CSW_SIZE_WORD    = 0x2U;
constexpr uint32_t CSW_ADDRINC_MASK = 0x3U << 4;
constexpr uint32_t CSW_ADDRINC_OFF  = 0x0U << 4;
constexpr uint32_t CSW_ADDRINC_SINGLE = 0x1U << 4;

// TAR auto-increment is only architecturally defined within the 1kB region the
// address is in; past that boundary the client has to reload TAR, and so do we.
constexpr uint32_t TAR_AUTOINC_REGION = 1024U;

// Transfer request bits and the successful ack, repeated from the CMSIS-DAP
// DAP_TRANSFER_* encodings so callers need not pull in the protocol header.
// DAP.cpp static_asserts that they still agree.
constexpr uint32_t REQ_APnDP = 1U << 0;
constexpr uint32_t REQ_RnW   = 1U << 1;
constexpr uint32_t REQ_ADDR  = 0x0CU;          // A[3:2], i.e. the register offset
constexpr uint8_t  ACK_OK    = 1U << 0;

// Bit 7 of a sequence info byte: capture the line instead of driving it.
constexpr uint8_t SEQ_CAPTURE = 0x80U;

// Clock out a raw sequence with SWDIO driven throughout -- the switch
// sequences that get a debug port's attention, where there is no packet
// framing at all. `bits` clock cycles are taken from `data`, low bit of each
// byte first.
void sequence_swj(uint32_t bits, const uint8_t *data);

// One DAP_SWD_Sequence-style phase: `info` carries the clock count in bits
// 5..0 (zero meaning 64) and SEQ_CAPTURE in bit 7. Captured bits land in `in`,
// driven bits come from `out`; the unused one may be null. This is the way to
// build a transfer whose response the target will not drive -- writing DP
// TARGETSEL on a multi-drop wire being the one that matters here.
void sequence_swd(uint8_t info, const uint8_t *out, uint8_t *in);

// One SWD transfer, retried while the target answers WAIT (up to the retry
// count the client set with DAP_TransferConfigure). `request` is the CMSIS-DAP
// DAP_TRANSFER_* encoding -- APnDP | RnW | A[3:2] -- and the return value is
// the DAP_TRANSFER_* ack, DAP_TRANSFER_OK on success. `data` is the value to
// write, the place to put the value read, or NULL to post a read whose result
// a later transfer will collect.
uint8_t transfer(uint32_t request, uint32_t *data);

// The client's view of the registers we would otherwise change out from under
// it. A field is only valid once the client has actually written it: a client
// that has not written SELECT yet has no cached value to protect, because it
// must write one before its own next access.
struct Shadow {
  bool     select_valid;
  uint32_t select;

  bool     csw_valid;
  uint8_t  csw_ap;          // APSEL the CSW value belongs to
  uint32_t csw;

  bool     tar_valid;
  uint8_t  tar_ap;          // APSEL the TAR value belongs to
  uint32_t tar;             // tracked through the AP's own auto-increment
};

// Read the shadow, and stop/resume updating it. A probe-side burst snapshots
// the shadow, suspends tracking so its own transfers are not mistaken for the
// client's, and restores the hardware from the snapshot before resuming.
const Shadow &shadow();
void snoop_suspend();
void snoop_resume();

// Forget everything. The client's own cache is invalid across a connect or a
// disconnect too, so there is nothing left to protect at those points.
void shadow_reset();

// Whether the debug port is in SWD mode, i.e. the client has run DAP_Connect.
// Probe-side transfers are only meaningful once it has.
bool port_is_swd();

}  // namespace swd

#endif  // SWD_PORT_H
