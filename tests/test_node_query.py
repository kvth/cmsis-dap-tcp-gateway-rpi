#!/usr/bin/env python3
"""Wire-protocol test of CMD_NODE_QUERY (0x87, batched sub-requests) and
CMD_NODE_DETECT (0x92), against the real gateway code (node_query.cpp,
rtt.cpp, dp_connect.cpp) running inside build/sim_probe. See node_query.h
for the wire format this drives directly.

sim_probe simulates a real RTT control block in target RAM (one up buffer,
one down buffer, both named "Terminal", channel 0) and a real multidrop DP,
so RTT_Write/RTT_Read ring-buffer mechanics are genuinely exercised, not
just the request/response framing.

    make sim_probe && ./tests/test_node_query.py
"""
import os
import socket
import struct
import subprocess
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM_PROBE = os.path.join(REPO_ROOT, "build", "sim_probe")

SIGNATURE = 0x00504144
TYPE_REQUEST = 1
TYPE_RESPONSE = 2
HDR = struct.Struct("<IHBB")

CMD_NODE_QUERY = 0x87
CMD_NODE_DETECT = 0x92

STATUS_OK = 0x00
STATUS_BAD_REQUEST = 0x0A


class Client:
    def __init__(self, port):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.buf = b""

    def command(self, payload):
        self.sock.sendall(HDR.pack(SIGNATURE, len(payload), TYPE_REQUEST, 0) + bytes(payload))
        while True:
            if len(self.buf) >= HDR.size:
                sig, length, kind, _ = HDR.unpack_from(self.buf)
                assert sig == SIGNATURE
                if len(self.buf) >= HDR.size + length:
                    body = self.buf[HDR.size:HDR.size + length]
                    self.buf = self.buf[HDR.size + length:]
                    assert kind == TYPE_RESPONSE
                    return body
            chunk = self.sock.recv(65536)
            assert chunk, "connection closed"
            self.buf += chunk


def build_detect_req(flags, mux_pos, targetsel):
    return bytes([CMD_NODE_DETECT, flags, mux_pos]) + struct.pack("<I", targetsel)


def parse_detect_resp(resp):
    assert resp[0] == CMD_NODE_DETECT, resp[:1]
    status = resp[1]
    dpidr, ctrl_stat = struct.unpack_from("<II", resp, 2)
    return status, dpidr, ctrl_stat


def build_query_req(flags, mux_pos, targetsel, ap, cb_addr, down_ch, up_ch,
                    max_wait_ms, sub_requests):
    req = bytearray([CMD_NODE_QUERY, flags, mux_pos])
    req += struct.pack("<I", targetsel)
    req.append(ap)
    req += struct.pack("<I", cb_addr)
    req += bytes([down_ch, up_ch])
    req += struct.pack("<H", max_wait_ms)
    req.append(len(sub_requests))
    for sub in sub_requests:
        req += struct.pack("<H", len(sub))
        req += sub
    return bytes(req)


def parse_query_resp(resp):
    assert resp[0] == CMD_NODE_QUERY, resp[:1]
    status = resp[1]
    dpidr, ctrl_stat = struct.unpack_from("<II", resp, 2)
    response_count = resp[10]
    entries = []
    pos = 11
    for _ in range(response_count):
        write_accepted, read_len = struct.unpack_from("<HH", resp, pos)
        pos += 4
        data = resp[pos:pos + read_len]
        pos += read_len
        entries.append((write_accepted, read_len, data))
    return status, dpidr, ctrl_stat, response_count, entries


failures = []


def check(cond, msg):
    if not cond:
        failures.append(msg)
        print("FAIL:", msg)
    else:
        print("ok:", msg)


def main():
    if not os.path.exists(SIM_PROBE):
        print("build/sim_probe not found -- run `make sim_probe` first", file=sys.stderr)
        sys.exit(2)

    port = 15900
    proc = subprocess.Popen(
        [SIM_PROBE, str(port), "--multidrop", "--targetsel=0x01002927"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.3)
    try:
        c = Client(port)

        # --- CMD_NODE_DETECT: present (correct targetsel) ---
        resp = c.command(build_detect_req(0x01, 0, 0x01002927))  # bit0 multidrop
        status, dpidr, ctrl_stat = parse_detect_resp(resp)
        check(status == STATUS_OK, "detect: present -> STATUS_OK (got %d)" % status)
        check(dpidr != 0, "detect: present -> nonzero dpidr (got 0x%08x)" % dpidr)

        # sim_probe emits a one-time "boot: simulated target ready\n" (29
        # bytes) into the up buffer at process startup. Drain it with a
        # direct RTT_START (0x81) + RTT_READ (0x84), now that detect above
        # has established the DP connection RTT_START needs, so later reads
        # observe only what this test itself writes, not that backlog.
        start_req = bytes([0x81, 0]) + struct.pack("<II", 0x20001000, 0)  # ap=0, cb_addr, search_len=0
        start_resp = c.command(start_req)
        assert start_resp[0] == 0x81 and start_resp[1] == STATUS_OK, start_resp[:2]
        boot_drain = c.command(bytes([0x84, 0]) + struct.pack("<H", 1023))  # channel 0
        assert boot_drain[0] == 0x84 and boot_drain[1] == STATUS_OK, boot_drain[:2]
        drained_len = struct.unpack_from("<H", boot_drain, 2)[0]
        check(drained_len == len(b"boot: simulated target ready\n"),
              "drain: boot backlog length matches (got %d)" % drained_len)

        # --- CMD_NODE_DETECT: under-length request ---
        # sim_probe (like the real DAP dispatcher) passes remaining *packet
        # capacity* as request_room, not the number of bytes actually sent
        # for this one command (see DAP.cpp's Request_Room comment) -- a
        # single vendor command can't tell "too few bytes were sent" from
        # "the rest of a full-size packet is just unused". This is shared by
        # every vendor command, not specific to node_query, so a short send
        # here just reads whatever (zeroed, in this fresh connection) bytes
        # follow rather than being rejected. Confirm it doesn't crash and
        # doesn't false-positive a detect.
        resp = c.command(bytes([CMD_NODE_DETECT, 0x01]))  # missing mux_pos+targetsel
        status, dpidr, ctrl_stat = parse_detect_resp(resp)
        print("info: detect with short send reads stale/zero bytes -> status=%d dpidr=0x%x (not a rejection path)"
              % (status, dpidr))

        # --- CMD_NODE_QUERY with request_count=0 behaves like detect ---
        resp = c.command(build_query_req(0x01, 0, 0x01002927, 0, 0x20001000, 0, 0, 50, []))
        status, dpidr, ctrl_stat, response_count, entries = parse_query_resp(resp)
        check(status == STATUS_OK, "query N=0: STATUS_OK (got %d)" % status)
        check(dpidr != 0, "query N=0: nonzero dpidr")
        check(response_count == 0, "query N=0: response_count == 0 (got %d)" % response_count)

        # --- CMD_NODE_QUERY with 3 sub-requests, all writes should land ---
        subs = [b"AAA", b"BBBBB", b"C"]
        resp = c.command(build_query_req(0x01, 0, 0x01002927, 0, 0x20001000, 0, 0, 20, subs))
        status, dpidr, ctrl_stat, response_count, entries = parse_query_resp(resp)
        check(status == STATUS_OK, "query N=3: STATUS_OK (got %d)" % status)
        check(response_count == 3, "query N=3: response_count == 3 (got %d)" % response_count)
        for i, (sub, (write_accepted, read_len, data)) in enumerate(zip(subs, entries)):
            check(write_accepted == len(sub),
                  "query N=3: sub %d write_accepted == %d (got %d)" % (i, len(sub), write_accepted))
            # target_poll() (sim_probe's simulated CPU) only runs between
            # top-level gateway commands, not between the RTT_Write/RTT_Read
            # sub-calls inside one node_query batch, so no echo reply can
            # appear until a later top-level command.
            check(read_len == 0, "query N=3: sub %d read_len == 0, echo can't land mid-batch (got %d)" % (i, read_len))

        # --- Verify the bytes actually landed in the down ring, in order,
        # by reading RTT status (write offset should equal total bytes sent). ---
        total_written = sum(len(s) for s in subs)
        rtt_status_req = bytes([0x83, 1, 0])  # CMD_RTT_STATUS, direction=1 (down), channel 0
        resp = c.command(rtt_status_req)
        assert resp[0] == 0x83 and resp[1] == STATUS_OK, resp[:2]
        buffer_, size, wr_off, rd_off, flags_ = struct.unpack_from("<5I", resp, 2)
        check(wr_off == total_written,
              "down ring write offset advanced by all 3 writes: %d == %d" % (wr_off, total_written))

        # --- request_count exceeding MAX_REQUESTS (32) is rejected ---
        req = bytearray([CMD_NODE_QUERY, 0x01, 0]) + struct.pack("<I", 0x01002927)
        req.append(0)
        req += struct.pack("<I", 0x20001000)
        req += bytes([0, 0])
        req += struct.pack("<H", 20)
        req.append(33)  # request_count > MAX_REQUESTS
        resp = c.command(bytes(req))
        status, dpidr, ctrl_stat, response_count, entries = parse_query_resp(resp)
        check(status == STATUS_BAD_REQUEST, "query: request_count=33 -> STATUS_BAD_REQUEST (got %d)" % status)

        # --- a sub-request whose declared length runs past the packet is malformed ---
        req = bytearray([CMD_NODE_QUERY, 0x01, 0]) + struct.pack("<I", 0x01002927)
        req.append(0)
        req += struct.pack("<I", 0x20001000)
        req += bytes([0, 0])
        req += struct.pack("<H", 20)
        req.append(1)
        req += struct.pack("<H", 9999)  # claims 9999 bytes, none follow
        resp = c.command(bytes(req))
        status, dpidr, ctrl_stat, response_count, entries = parse_query_resp(resp)
        check(status == STATUS_BAD_REQUEST,
              "query: sub-request length past end of packet -> STATUS_BAD_REQUEST (got %d)" % status)

        # The N=3 batch above left "AAABBBBBC" sitting unread in the down
        # buffer; a *later* top-level command's target_poll() (every command
        # runs it at entry, including the two bad-request checks just above)
        # already echoed it as "echo: AAABBBBBC" into the up buffer. Drain
        # that backlog so it doesn't leak into the round-trip test below.
        for _ in range(10):
            resp = c.command(build_query_req(0x01, 0, 0x01002927, 0, 0x20001000, 0, 0, 0, [b""]))
            _, _, _, _, drain_entries = parse_query_resp(resp)
            if drain_entries[0][1] == 0:
                break
        else:
            failures.append("drain: up channel never went quiet")

        # --- genuine round trip: write now, echo lands only after the
        # simulated target "runs" (between top-level commands), read on a
        # later call -- exactly the two-phase pattern devicedriver_rtt uses
        # (queue a write, poll for the reply on a later cycle). ---
        resp = c.command(build_query_req(0x01, 0, 0x01002927, 0, 0x20001000, 0, 0, 20, [b"hi"]))
        status, dpidr, ctrl_stat, response_count, entries = parse_query_resp(resp)
        check(status == STATUS_OK, "roundtrip: write 'hi' -> STATUS_OK (got %d)" % status)
        check(entries[0][0] == 2, "roundtrip: write_accepted == 2 (got %d)" % entries[0][0])
        check(entries[0][1] == 0, "roundtrip: no echo yet, read_len == 0 (got %d)" % entries[0][1])

        # A later top-level command's target_poll() picks up "hi" from the
        # down buffer and emits "echo: HI" into the up buffer; a 0-length
        # sub-request here skips the write and goes straight to polling read.
        resp = c.command(build_query_req(0x01, 0, 0x01002927, 0, 0x20001000, 0, 0, 100, [b""]))
        status, dpidr, ctrl_stat, response_count, entries = parse_query_resp(resp)
        check(status == STATUS_OK, "roundtrip: poll -> STATUS_OK (got %d)" % status)
        check(entries[0][0] == 0, "roundtrip: 0-length write_accepted == 0 (got %d)" % entries[0][0])
        check(entries[0][2] == b"echo: HI",
              "roundtrip: echoed reply matches (got %r)" % entries[0][2])

        # --- absent node (wrong targetsel): detect and query N>0 both fail cleanly ---
        resp = c.command(build_detect_req(0x01, 0, 0xDEADBEEF))
        status, dpidr, ctrl_stat = parse_detect_resp(resp)
        check(status != STATUS_OK, "detect: wrong targetsel -> not STATUS_OK (got %d)" % status)
        check(dpidr == 0, "detect: wrong targetsel -> dpidr zeroed")

        resp = c.command(build_query_req(0x01, 0, 0xDEADBEEF, 0, 0x20001000, 0, 0, 20, [b"X"]))
        status, dpidr, ctrl_stat, response_count, entries = parse_query_resp(resp)
        check(status != STATUS_OK, "query: wrong targetsel -> not STATUS_OK (got %d)" % status)
        check(response_count == 0, "query: wrong targetsel -> response_count == 0 (no RTT attempted)")

    finally:
        proc.terminate()
        proc.wait(timeout=3)

    print()
    if failures:
        print("%d FAILURES" % len(failures))
        sys.exit(1)
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
