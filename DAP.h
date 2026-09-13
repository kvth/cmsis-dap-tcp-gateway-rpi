/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CMSIS-DAP command processing: the interface the TCP server and startup code
 * use. The protocol constants and the SWD bitbang behind it are private to
 * DAP.cpp (see DAP_protocol.h and DAP_config.h).
 */

#ifndef DAP_H
#define DAP_H

#include <stddef.h>
#include <stdint.h>

namespace cmsis {
namespace dap {

// Largest request or response body, in bytes. Callers size their buffers with
// this; DAP.cpp static_asserts that it matches the DAP_PACKET_SIZE the command
// handlers were built with.
constexpr size_t MAX_PACKET_SIZE = 1024;

// How many packets we tell the client it may have outstanding (DAP_Info's
// packet count). A client that believes us can put this many requests on the
// wire before waiting for a response, so the receive buffer is sized from it.
// DAP.cpp static_asserts it against DAP_PACKET_COUNT.
constexpr size_t MAX_PACKET_COUNT = 8;

// A single command's response can be several times the size of the request
// that produced it: a DAP_Transfer read with timestamps emits 8 response bytes
// (4 data + 4 timestamp) per request byte, and DAP_ExecuteCommands can pack a
// request full of them. The command handlers inherited from ARM's CMSIS-DAP do
// not bounds-check their own output, so callers must hand process_cmd() /
// execute_cmd() a response buffer of MAX_PACKET_SIZE * this, not just
// MAX_PACKET_SIZE, and reject an over-long result afterwards.
constexpr size_t MAX_RESPONSE_AMPLIFICATION = 8;

// SWD clock calibration: a two-parameter linear model for how many bare
// delay-loop iterations one SWCLK half-period needs --
//
//   delay_iterations = ceil(speed_coeff / khz) - speed_offset
//
// speed_offset is the fixed cost of one bit transition (the GPIO register
// writes themselves) expressed in delay-loop iterations, and speed_coeff turns
// a requested period into iterations. The fastest clock the bitbang can reach
// is therefore speed_coeff / speed_offset kHz, where the loop runs zero times.
//
// The defaults below are a published Pi 2 measurement for that same model
// (OpenOCD's tcl/interface/jtag_hat_rpi2.cfg: "bcm2835gpio speed_coeffs
// 146203 36" -- OpenOCD's own bitbang driver, src/jtag/drivers/bcm2835gpio.c,
// solves the identical timing problem this way), used here as a starting
// point rather than measured fresh. They only hold to the extent this
// program's own delay loop -- PIN_DELAY_SLOW() in DAP.cpp, an independent
// implementation of the same idiom -- costs the same per iteration, which
// is not guaranteed on a different compiler or Pi model; re-measure per
// board, which is what --speed-coeff/--speed-offset are for, or
// calibrate() below to measure them automatically.
constexpr unsigned int SPEED_COEFF_DEFAULT  = 146203;
constexpr unsigned int SPEED_OFFSET_DEFAULT = 36;

// SWD clock to start up with, in kHz. This only governs the window before the
// client picks a clock of its own: OpenOCD sends DAP_SWJ_Clock during init in
// response to "adapter speed", and that overrides this. It is also the clock a
// client that never asks will get, so it sits comfortably below the
// speed_coeff / speed_offset ceiling rather than at it.
constexpr unsigned int CLOCK_KHZ_DEFAULT = 1000;

// Set up the DAP defaults and the debug I/O pins. The pin mapping must already
// be in place (see gpio::configure_split() / gpio::configure_single()).
//   clock_hz: startup SWD clock, in Hertz; see CLOCK_KHZ_DEFAULT above.
void init(unsigned int speed_coeff = SPEED_COEFF_DEFAULT,
          unsigned int speed_offset = SPEED_OFFSET_DEFAULT,
          unsigned int clock_hz = CLOCK_KHZ_DEFAULT * 1000U);

// Release the DAP I/O pins and unmap the GPIO registers. Safe to call even
// if init() was never called.
void shutdown();

// Self-calibrate speed_coeff/speed_offset for this machine, in place of
// trusting the Pi 2 measurements SPEED_COEFF_DEFAULT/SPEED_OFFSET_DEFAULT
// were taken from. Times the delay loop and a bare SWCLK register write
// directly (see calibrate() in DAP.cpp), so it needs no target on the wire
// and no scope -- only the GPIO pins to already be memory-mapped, i.e.
// gpio::configure_split()/configure_single() must have run first.
//
// The measured values are applied immediately (as if passed to init(), or to
// a fresh Set_Clock_Delay() call at whatever clock is currently in effect)
// and also handed back through the two out-params, so a caller that started
// from init()'s defaults picks up the same numbers init() would have used.
// Safe to call again later, including while a client is connected: it drives
// real SWCLK edges with SWDIO left alone, which any SWD target tolerates the
// same way it tolerates a line reset's clocking.
//   speed_coeff_out, speed_offset_out: filled in on success; untouched on
//                                      failure.
//   return: false if gpio pins are not mapped yet.
bool calibrate(unsigned int *speed_coeff_out, unsigned int *speed_offset_out);

// Process a single DAP command request and prepare the response.
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
uint32_t process_cmd(const uint8_t *request, uint8_t *response);

// Execute a DAP command, resolving DAP_QueueCommands/DAP_ExecuteCommands
// into the individual commands they carry.
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
uint32_t execute_cmd(const uint8_t *request, uint8_t *response);

}  // namespace dap
}  // namespace cmsis

#endif  // DAP_H
