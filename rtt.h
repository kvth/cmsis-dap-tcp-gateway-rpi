/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * SEGGER RTT over CMSIS-DAP vendor commands.
 *
 * RTT is a ring buffer in target RAM. A control block names the buffers; the
 * target writes into an "up" buffer and reads from a "down" buffer, and the
 * debug probe moves the read and write offsets along by reading and writing
 * target memory over the MEM-AP while the CPU keeps running. Nothing about it
 * needs the target to be halted.
 *
 * Normally the host does all of that itself, one CMSIS-DAP transfer command at
 * a time. Over TCP that is expensive: every offset read and every data chunk
 * is a network round trip, and the ring pointer dance needs several of them
 * per poll. These vendor commands move the whole exchange onto the probe, so
 * one request and one response cover a poll of a channel.
 *
 *
 * WIRE FORMAT
 *
 * Multi-byte values are little endian, as everywhere else in CMSIS-DAP. Every
 * response begins with the command ID echoed back, then a one-byte status
 * from vendor::Status. On a failure the fields after the status byte are
 * still present but meaningless, except where noted.
 *
 *   0x81 RTT_Start
 *     request:   u8 ap, u32 address, u32 search_len
 *     response:  u8 status, u32 cb_addr, u8 num_up, u8 num_down
 *
 *     Locates the control block on MEM-AP `ap` and remembers it. With
 *     search_len == 0, `address` is the control block itself. Otherwise
 *     [address, address+search_len) is scanned for the RTT ID string, which
 *     is how a host that only knows the target's RAM range finds it. Ranges
 *     that are partly unmapped are fine; the scan steps over the faults.
 *
 *   0x82 RTT_Stop
 *     request:   -
 *     response:  u8 status
 *
 *   0x83 RTT_Status
 *     request:   u8 direction (0 = up, 1 = down), u8 channel
 *     response:  u8 status, u32 buffer, u32 size, u32 write_off,
 *                u32 read_off, u32 flags, u8 name_len, char name[name_len]
 *
 *     The channel's descriptor as it stands right now. `name` is the target's
 *     channel name, truncated to NAME_MAX bytes.
 *
 *   0x84 RTT_Read
 *     request:   u8 channel, u16 max_len
 *     response:  u8 status, u16 len, u8 data[len]
 *
 *     Drains up to max_len bytes from an up buffer and advances the target's
 *     read offset by what it took. len == 0 with status ok means the channel
 *     had nothing pending. The response is capped at what is left of the
 *     packet, so a batched request gets less than it asked for rather than an
 *     oversized response the client would drop.
 *
 *   0x85 RTT_Write
 *     request:   u8 channel, u16 len, u8 data[len]
 *     response:  u8 status, u16 accepted
 *
 *     Appends to a down buffer, as much of `data` as fits. `accepted` is how
 *     much went in; a caller with more to send retries with the remainder,
 *     since the buffer only drains as the target reads it.
 *
 *
 * CONCURRENCY WITH THE CLIENT
 *
 * These commands drive the DP and the AP themselves, on a wire the client is
 * also using. target_mem::Burst is what keeps that from corrupting the
 * client's session -- see swd_port.h. The client still has to have brought
 * the debug port up (DAP_Connect, power-up request, an AP that reaches RAM);
 * these commands do not initialise the DAP, they borrow it.
 */

#ifndef RTT_H
#define RTT_H

#include <stddef.h>
#include <stdint.h>

#include "vendor.h"

namespace rtt {

// Vendor command IDs and the shared status byte live in vendor.h:
// CMD_RTT_START through CMD_RTT_WRITE, and vendor::Status.

// Longest channel name RTT_Status will return.
constexpr size_t NAME_MAX = 32;

// Whether `cmd` is one of the commands above.
bool handles(uint8_t cmd);

// Process one RTT vendor command. `request` points at the command ID, and
// `response` at where the echoed ID goes.
//
// `request_room` and `response_room` are how many bytes of the request and
// response packets are still unread and unwritten. These commands are the
// only variable-length ones here, so they are also the only ones that would
// otherwise take a client's word for a length: the request bound is what
// keeps a truncated or overstated RTT_Write inside its packet, and the
// response bound is what keeps RTT_Read's answer inside its own.
//
// Returns the DAP handler encoding: request length in the upper 16 bits,
// response length in the lower 16.
uint32_t handle_command(const uint8_t *request, uint8_t *response,
                        size_t request_room, size_t response_room);

// Drop the located control block. The client's own view of the target is
// invalid across a DAP connect or disconnect, and so is ours.
void reset();

}  // namespace rtt

#endif  // RTT_H
