/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RP2040 flash programming as vendor commands.
 *
 * The RP2040 has no on-chip flash controller a debugger could drive. Its
 * bootrom instead exports the routines that talk to the external QSPI part,
 * and a host programs flash by halting the core, pointing it at one of those
 * routines and letting it run. OpenOCD does exactly that in
 * src/flash/nor/rp2040.c (now rp2xxx.c): look the routine up in the bootrom's
 * function table, set r0..r3 to the arguments and r7 to the routine, run the
 * bootrom's own debug trampoline, and wait for the breakpoint at the end of it.
 *
 * Doing that from the far side of a CMSIS-DAP link is a lot of round trips.
 * One ROM call is seven core-register writes -- and a core register write is
 * itself two memory writes plus a poll -- followed by a halt poll; a 256-byte
 * page is another 64 memory writes on top. Over TCP, at one packet per
 * transfer, programming a few hundred kB takes hundreds of thousands of round
 * trips. These commands move the whole dance onto the probe, the same way the
 * RTT commands move the ring buffer protocol onto it (see rtt.h): the client
 * sends flash data and gets back a status.
 *
 *
 * THE SESSION
 *
 * RP_Attach halts the core, checks the bootrom and reads the function table,
 * and is what every other command here needs first. After that a flashing
 * session mirrors OpenOCD's, which brackets flash work between a prep and a
 * finish rather than paying for both on every chunk:
 *
 *   RP_Attach                     halt, find the bootrom entry points
 *   RP_Flash_Prep                 connect_internal_flash + flash_exit_xip
 *   RP_Flash_Erase                flash_range_erase, repeat per range
 *   RP_Flash_Stage                fill the probe's staging buffer in target RAM
 *   RP_Flash_Program              flash_range_program out of it
 *   RP_Flash_Finish               flash_flush_cache + flash_enter_cmd_xip
 *   RP_Flash_CRC                  read back and checksum, now that XIP is on
 *
 * Between Prep and Finish the QSPI interface is in direct command mode, so the
 * flash is not memory mapped and the target cannot execute from it. Nothing
 * but Finish puts that back, including a client that disconnects half way: a
 * session that dies in the middle leaves a target that will not boot until
 * something flashes it again. That is the same exposure OpenOCD has.
 *
 * The other core is not addressed here, and on a two-core part that matters:
 * while flash is out of the map, code running on core 1 out of XIP faults.
 * RP_Attach's reset-halt flag is the answer, because SYSRESETREQ on an RP2040
 * resets the whole chip -- core 1 comes out of it parked in the bootrom
 * waiting on its mailbox, not executing flash. Attaching to a running target
 * with a plain halt only stops the core you named.
 *
 * These commands also trash the target. The core's registers are set to run
 * bootrom code, the staging buffer and the ROM stack overwrite target RAM, and
 * neither is put back. That is what a flashing session is; an RTT session on
 * the same target has to be re-established afterwards.
 *
 * Like the RTT commands, these borrow a debug port the client has already
 * brought up -- DAP_Connect and then DP_Connect (see dp_connect.h), or the
 * client's own equivalent. They do not initialise the DAP.
 *
 *
 * WIRE FORMAT
 *
 * Little endian throughout. Every response is the command ID echoed back, then
 * a vendor::Status byte, then the fields below; on a failure those fields are
 * present but zero.
 *
 *   0x88 RP_Attach
 *     request:   u8 ap, u8 flags, u32 stack_top, u32 staging_addr,
 *                u32 staging_len
 *     response:  u8 status, u32 dhcsr, u32 rom_magic, u16 jump_table[8]
 *
 *     `ap` is the MEM-AP that reaches the core -- AP 0 for RP2040 core 0, AP 1
 *     for core 1. Either can flash, as both see the same bootrom and the same
 *     SRAM, so a client that only wants to program flash can stay on core 0.
 *
 *     flags bit 0   halt the core before doing anything, and wait for it.
 *     flags bit 1   reset the core and catch it halted out of reset, rather
 *                   than halting it where it stands. Needed when what is
 *                   running has reconfigured the QSPI pins or is sitting in a
 *                   WFI the ROM calls would not survive; harmless otherwise.
 *                   Implies bit 0.
 *
 *     `stack_top`, `staging_addr` and `staging_len` place the stack the ROM
 *     routines run on and the buffer RP_Flash_Stage fills, both in target RAM.
 *     Zero means the default: a stack at the top of SRAM5 and a 64 kB staging
 *     buffer at 0x20020000. They are remembered until the next RP_Attach.
 *
 *     `rom_magic` is word 0x00000010, 'M' 'u' and a version byte, and is what
 *     says this is an RP2040 at all; STATUS_NO_BOOTROM if it is not.
 *     `jump_table` is, in order: debug_trampoline, debug_trampoline_end,
 *     connect_internal_flash, flash_exit_xip, flash_range_erase,
 *     flash_range_program, flash_flush_cache, flash_enter_cmd_xip. The client
 *     does not need them -- the commands below look them up themselves -- but
 *     they identify the bootrom revision and make a failure diagnosable.
 *
 *   0x89 RP_Core
 *     request:   u8 action, u16 timeout_ms
 *     response:  u8 status, u32 dhcsr
 *
 *     action 0 query, 1 halt, 2 resume, 3 reset and run, 4 reset and halt.
 *     `dhcsr` is the debug status as it stands when the action has settled.
 *
 *   0x8A RP_Call
 *     request:   u8 n_args, u16 func, u16 timeout_ms, u32 args[4]
 *     response:  u8 status, u32 r0
 *
 *     Run bootrom entry point `func` through the debug trampoline with the
 *     first `n_args` of `args` in r0..r3, and return r0. The escape hatch for
 *     a bootrom routine these commands do not wrap; `args` is always four
 *     words on the wire whatever `n_args` says, so the request is fixed size.
 *
 *   0x8B RP_Flash_Prep
 *     request:   -
 *     response:  u8 status
 *
 *     connect_internal_flash then flash_exit_xip. Leaves the flash in direct
 *     command mode and not memory mapped -- see THE SESSION above.
 *
 *   0x8C RP_Flash_Erase
 *     request:   u32 addr, u32 count, u32 block_size, u8 block_cmd,
 *                u16 timeout_ms
 *     response:  u8 status
 *
 *     flash_range_erase. `addr` and `count` are byte offsets into flash, both
 *     multiples of 4096. `block_size` and `block_cmd` are the larger erase the
 *     ROM may use where a range covers a whole block: 65536 and 0xD8 suit the
 *     parts Raspberry Pi ship, and block_size 0 restricts it to 4 kB sectors.
 *
 *   0x8D RP_Flash_Stage
 *     request:   u32 offset, u16 len, u8 data[len]
 *     response:  u8 status, u16 written
 *
 *     Put `data` at `offset` in the staging buffer. Splitting staging from
 *     programming is what keeps the ROM calls rare: a client fills as much of
 *     the buffer as it has data for, a packet at a time, and then programs the
 *     lot with one call instead of one per packet.
 *
 *   0x8E RP_Flash_Program
 *     request:   u32 addr, u32 stage_off, u32 count, u16 timeout_ms
 *     response:  u8 status
 *
 *     flash_range_program from `stage_off` in the staging buffer to byte
 *     offset `addr` in flash. `addr` and `count` are multiples of 256, the
 *     ROM's page size. What is not staged is not written: the client pads the
 *     tail of its last page itself, with 0xFF to match erased flash.
 *
 *   0x8F RP_Flash_Finish
 *     request:   u8 flags
 *     response:  u8 status
 *
 *     flash_flush_cache then flash_enter_cmd_xip, which is what makes flash
 *     memory mapped and executable again. flags bit 0 also resumes the core.
 *     Both ROM calls are attempted even if the first fails, so a flush that
 *     goes wrong still leaves a target that boots.
 *
 *   0x90 RP_Flash_CRC
 *     request:   u32 addr, u32 count
 *     response:  u8 status, u32 crc32
 *
 *     CRC-32 (the zlib/IEEE one: reflected, polynomial 0xEDB88320, initial and
 *     final value 0xFFFFFFFF inverted) over `count` bytes at byte offset
 *     `addr`, read through the XIP window. Verifying a write by reading it
 *     back over the link costs a packet per few hundred bytes; this costs one
 *     packet per range. Only meaningful once RP_Flash_Finish has restored XIP.
 *     `count` is capped at CRC_MAX_LEN so a verify cannot monopolise the
 *     gateway, and a larger range is the client's to split.
 */

#ifndef RP2040_H
#define RP2040_H

#include <stddef.h>
#include <stdint.h>

#include "vendor.h"

namespace rp2040 {

// Most a single RP_Flash_CRC may checksum. Reading flash over a bitbanged SWD
// link is the slow part, and one command should not hold the gateway for
// longer than a client's own read timeout.
constexpr uint32_t CRC_MAX_LEN = 64U * 1024U;

// Longest any of these will wait on the target, whatever a request asks for.
// Erasing a whole 2 MB part takes longer than this, so a client erases in
// ranges; that also keeps the link from looking hung.
constexpr uint16_t MAX_TIMEOUT_MS = 10000U;

// Whether `cmd` is one of the commands above.
bool handles(uint8_t cmd);

// Process one. Arguments and return value follow rtt::handle_command().
uint32_t handle_command(const uint8_t *request, uint8_t *response,
                        size_t request_room);

// Forget the attached session. As with rtt::reset(), what we knew about the
// target does not survive a connect or a disconnect.
void reset();

}  // namespace rp2040

#endif  // RP2040_H
