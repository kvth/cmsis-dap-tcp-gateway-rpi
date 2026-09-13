/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fused "is a node there, and if so run these RTT commands" vendor
 * commands.
 *
 * Sweeping a bank of nodes behind an SWD mux for RTT means running mux
 * select, DP connect, RTT start, RTT write and RTT read for every position.
 * Each of those is already one round trip on its own (swdmux.h,
 * dp_connect.h, rtt.h), but a sweep still pays one round trip per step per
 * node. This folds all of it into one round trip, by driving the existing
 * handlers with synthetic sub-requests rather than duplicating their logic.
 *
 * The two commands here mirror the two things a client sweeping a bank of
 * nodes typically checks every cycle: whether a node is there at all
 * (mux-select + DP connect, most cycles find nothing queued and stop right
 * there), and, only when something actually is queued for that position,
 * draining that queue's requests one at a time against it (each queued
 * item is its own write, then a wait for the matching reply). A client
 * built that way can now do both across one round trip each, however many
 * requests are actually queued that cycle -- zero most of the time,
 * occasionally a handful.
 *
 *
 * WIRE FORMAT
 *
 *   0x87 Node_Query
 *     request:   u8 flags, u8 mux_pos, u32 targetsel, u8 ap, u32 cb_addr,
 *                u8 down_channel, u8 up_channel, u16 max_wait_ms,
 *                u8 request_count,
 *                ( u16 len, u8 data[len] ) * request_count
 *     response:  u8 status, u32 dpidr, u32 ctrl_stat, u8 response_count,
 *                ( u16 write_accepted, u16 read_len, u8 read_data[read_len] )
 *                  * response_count
 *
 *     flags bit0     multidrop: pick targetsel before reading DPIDR, as
 *                    dp_connect.h's flags bit 0.
 *     flags bits2:1  switch mode, same encoding as dp_connect.h.
 *     flags bit3     skip the mux select and use whatever position the
 *                    gateway already has selected -- a bench rig with no
 *                    mux wired up.
 *
 *     cb_addr is used as-is, the way RTT_Start's search_len == 0 does: this
 *     never searches. Caching the address once and passing it in beats a
 *     fresh RAM scan on every node in a sweep.
 *
 *     request_count == 0 means "just tell me if it's there": mux select and
 *     DP connect run, RTT does not -- the same shortcut a polling client
 *     takes when nothing is queued for a position (its own DP/MEM-AP init
 *     and RTT start are skipped too). Otherwise RTT_Start runs once for the
 *     whole batch, and each of the request_count sub-requests is written in
 *     full (looped RTT_Write) and then polled for its own reply (looped
 *     RTT_Read, up to max_wait_ms) in order, producing one sub-response
 *     each. A sub-request whose write stalls (the ring is not draining)
 *     reports whatever write_accepted it managed with read_len 0 and moves
 *     on to the next one; only a genuine transport failure (DP_Connect,
 *     RTT_Start, or an RTT_Write/RTT_Read call itself reporting a status
 *     other than STATUS_OK) stops the batch early, in which case
 *     response_count is short of request_count and status carries the
 *     reason. Responses are matched to requests by the order they were
 *     sent in, not by anything in the node protocol's own sequence byte --
 *     this stays as protocol-agnostic as RTT_Read always has been, so a
 *     stale reply left over on the channel from outside this batch (say, a
 *     client that connected directly) can still land as read_data for
 *     whichever sub-request happened to be waiting when it showed up.
 *
 *     status is STATUS_MUX_FAILED if the mux itself refused the switch (a
 *     stale mux position would make a "connected" DPIDR meaningless), or
 *     whatever DP_Connect/RTT_Start/RTT_Write/RTT_Read reported for
 *     whichever step actually failed. dpidr and ctrl_stat stay 0 until
 *     DP_Connect succeeds.
 *
 *     The response is clamped to whatever is left of the packet: a batch
 *     whose replies do not all fit gets fewer completed sub-responses
 *     (response_count short of request_count, status still STATUS_OK)
 *     rather than one the client would drop whole.
 *
 *   0x92 Node_Detect
 *     request:   u8 flags, u8 mux_pos, u32 targetsel
 *     response:  u8 status, u32 dpidr, u32 ctrl_stat
 *
 *     Node_Query with request_count forced to 0, as a smaller fixed-size
 *     command for a caller that only ever wants the detect step -- no
 *     ap/cb_addr/channels/max_wait_ms to fill in. Same flags, same status
 *     meanings.
 */

#ifndef NODE_QUERY_H
#define NODE_QUERY_H

#include <stddef.h>
#include <stdint.h>

#include "vendor.h"

namespace node_query {

// Whether `cmd` is one of the two commands above.
bool handles(uint8_t cmd);

// Process one. `request` points at the command ID, and `response` at where
// the echoed ID goes. Arguments and return value follow rtt::handle_command().
uint32_t handle_command(const uint8_t *request, uint8_t *response,
                        size_t request_room, size_t response_room);

}  // namespace node_query

#endif  // NODE_QUERY_H
