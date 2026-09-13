/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * SWD clock self-calibration as a vendor command. See cmsis::dap::calibrate()
 * in DAP.h/.cpp for how the measurement itself works; this only wraps it in
 * the vendor command wire format.
 *
 *
 * WIRE FORMAT
 *
 *   0x91 Calibrate
 *     request:   (no arguments)
 *     response:  u8 status, u32 speed_coeff, u32 speed_offset
 *
 *     Re-measures the delay loop and a bare SWCLK register write on this
 *     machine, in place of the --speed-coeff/--speed-offset the process
 *     started with (which may just be SPEED_COEFF_DEFAULT/SPEED_OFFSET_DEFAULT,
 *     measured on a different Pi). Applies the result immediately, at
 *     whatever SWD clock is currently in effect, and reports the two numbers
 *     back so a client can log or persist them.
 *
 *     status is STATUS_NOT_MAPPED if the gpio pins are not memory-mapped
 *     yet -- not reachable in practice once the server is up, since that
 *     mapping happens at process startup before any command can arrive.
 *     speed_coeff/speed_offset stay 0 on failure.
 *
 *     This drives real SWCLK edges (SWDIO is left alone), the same as any
 *     line reset. Safe with a target already connected, but a session
 *     mid-transfer will see spurious clocking while it runs.
 */

#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <stddef.h>
#include <stdint.h>

#include "vendor.h"

namespace calibrate {

// Whether `cmd` is the command above.
bool handles(uint8_t cmd);

// Process it. Arguments and return value follow rtt::handle_command().
uint32_t handle_command(const uint8_t *request, uint8_t *response,
                        size_t request_room);

}  // namespace calibrate

#endif  // CALIBRATE_H
