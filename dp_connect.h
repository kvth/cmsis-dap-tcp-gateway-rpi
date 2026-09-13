/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Debug port bring-up as a vendor command.
 *
 * Getting a target's debug port talking is a fixed dance: clock out a switch
 * sequence to move the wire into SWD, on a multi-drop wire pick which DP will
 * answer, read DPIDR to confirm one did, clear the sticky error bits, and ask
 * for debug and system power until the target grants it. A host doing that
 * over CMSIS-DAP spends a dozen round trips on it, several of them raw bit
 * sequences it has to get exactly right. Over TCP that is slow, and every
 * client that wants to reach RTT without a full debugger reimplements it.
 *
 * This does the whole thing in one command, so a client's session is
 * DAP_Connect, DP_Connect, RTT_Start, and then it is reading.
 *
 *
 * WIRE FORMAT
 *
 * Two commands live here, both debug-port level: the bring-up, and the RP2040
 * rescue that undoes a target the bring-up can no longer reach.
 *
 *
 *   0x86 DP_Connect
 *     request:   u8 flags, u32 targetsel
 *     response:  u8 status, u32 dpidr, u32 ctrl_stat
 *
 *     flags bit 0     the wire is SWD multi-drop: select the DP named by
 *                     `targetsel` before looking for a DPIDR. An RP2040 needs
 *                     this -- 0x01002927 is its core 0, 0x11002927 its core 1,
 *                     and for reaching RTT either will do, as both cores see
 *                     the same SRAM. `targetsel` is ignored when this is clear.
 *     flags bits 2:1  which switch sequence to use: 0 tries JTAG-to-SWD and
 *                     then the dormant route, 1 forces JTAG-to-SWD, 2 forces
 *                     the dormant route.
 *
 *     `dpidr` identifies the debug port that answered, and `ctrl_stat` is
 *     DP CTRL/STAT as it stood once power was granted. Both are zero on
 *     failure.
 *
 *

 *   0x93 RP_Rescue
 *     request:   u8 flags, u32 targetsel
 *     response:  u8 status, u32 dpidr, u32 ctrl_stat
 *
 *     Put an RP2040 into rescue mode: reset the chip and stop it in the
 *     bootrom before it runs anything out of flash. For a board that has been
 *     flashed with firmware that hangs, reconfigures the QSPI pins or parks a
 *     core somewhere the debug port cannot follow, this is the way back --
 *     RP_Attach's reset-halt cannot help once the code being reset into is the
 *     problem.
 *
 *     Instance 0xF on the wire is not a core's debug port. Its CDBGPWRUPREQ is
 *     wired to the RP2040's power-on state machine: asserting it resets the
 *     chip with a flag set in VREG_AND_POR_CHIP_RESET, and clearing it again
 *     releases the reset, whereupon the bootrom sees that flag and halts. That
 *     is the sequence OpenOCD runs under `-c "set RESCUE 1"` in its
 *     target/rp2040.cfg, and this is the same one in a single round trip.
 *
 *     `targetsel` is the rescue port's, 0xF1002927 when zero -- the RP2040's
 *     part number with instance 0xF. flags bits 2:1 select the switch sequence
 *     exactly as DP_Connect's do; bit 0 is ignored, since a rescue is
 *     multi-drop by definition.
 *
 *     `ctrl_stat` is the readback that says it worked: every power request and
 *     acknowledge bit has to have gone, or the reset is still asserted and the
 *     status is STATUS_RESCUE_FAILED.
 *
 *     Afterwards the chip is in reset-then-bootrom state and nothing either
 *     side knew about it is true any more -- not the debug port, not the AP,
 *     and not the target's RAM. A client carries on with a fresh DP_Connect.
 *
 *
 * THIS ENDS WHATEVER SESSION WAS IN PROGRESS
 *
 * The RTT commands are built to borrow the wire from a live client and hand
 * it back untouched. This one is the opposite: a line reset and a fresh switch
 * sequence throw away the link the client was using, along with everything it
 * had cached about the DP and the AP. That is fine when this command *is* the
 * bring-up -- the server takes one client at a time, so if you are connected,
 * the session is yours. It is not fine sent into the middle of somebody else's
 * session, say from an OpenOCD console while OpenOCD is driving. Our own
 * cached state is dropped on the way in for the same reason.
 */

#ifndef DP_CONNECT_H
#define DP_CONNECT_H

#include <stddef.h>
#include <stdint.h>

#include "vendor.h"

namespace dp {

// Whether `cmd` is one of the commands above.
bool handles(uint8_t cmd);

// Process it. Arguments and return value follow rtt::handle_command().
uint32_t handle_command(const uint8_t *request, uint8_t *response,
                        size_t request_room);

}  // namespace dp

#endif  // DP_CONNECT_H
