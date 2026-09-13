# Tests

These run on any Linux host: there is no Raspberry Pi, no SWD wire and no
target board involved. The parts of the program that talk to hardware are
replaced by a simulated DP/MEM-AP over a block of ordinary memory, which is
enough to exercise everything the RTT support does.

    make test           build and run rtt_test, shadow_test and rp2040_test
    make sim_probe      build a fake probe to point a host client at
    make sim_probe && ./tests/test_node_query.py   exercise 0x87/0x92 over it

`rtt_test.cpp`
: The RTT ring buffer protocol and the MEM-AP access under it, against a
  simulated target holding a control block. Covers wrapping, unaligned
  buffers, full and empty rings, truncated requests, an AP without byte
  access, and the register save/restore that lets a command borrow the wire
  from a live client.

`shadow_test.cpp`
: The DP/AP shadow in DAP.cpp, which is what makes that borrowing safe.
  Includes the translation unit so the tracker and its state are reachable,
  with the GPIO registers pointed at a plain buffer.

`rp2040_test.cpp`
: The RP2040 flash vendor commands, against a simulated RP2040: SRAM, a
  bootrom holding the function table they look up, an XIP-mapped flash array
  and the ARMv6-M debug registers. There is no CPU -- the only code these
  commands ever set running is the bootrom's debug trampoline, so "resume"
  means "carry out the routine named in r7, then halt at the breakpoint".
  Covers the attach handshake, the alignment and bounds rules that stand in
  for bootrom asserts there is no way back from, the erase/stage/program/
  finish sequence, the XIP window being gone in the middle of it, the CRC
  read-back, and a trampoline call that goes astray.

`sim_probe.cpp`
: A fake probe: the real TCP framing, the real bring-up in dp_connect.cpp and
  the real RTT and RP2040 commands, over a simulated target whose channel
  echoes back what it is sent in upper case and whose bootrom programs a
  simulated flash. Useful for working on a host-side client, and the only
  place the bring-up runs at all without a board.

  It takes a port and then any of:

  `--multidrop`
  : model an SWD multi-drop wire, as an RP2040 has: after a line reset no DP
    answers until a TARGETSEL write picks one, and the DP that write picks
    answers nothing until the host reads DPIDR (ADIv5.2). That second rule is
    an easy step for a client to leave out and an obscure failure to debug
    without it -- no acknowledge on whatever transfer comes next, and nothing
    to say why -- so it is modelled here.

  `--targetsel=VALUE`
  : which DP the multi-drop wire will answer for (default 0x01002927, RP2040
    core 0).

  `--nopower`
  : never acknowledge the power-up request, to exercise the bring-up timeout.

      ./build/sim_probe 4441 --multidrop &

  It answers `DAP_Info` (including the SWD capability bit and the packet
  size), `DAP_Transfer` and `DAP_TransferBlock` as well as the vendor
  commands, and under `--multidrop` presents a DPv2 DPIDR and a banked
  DLPIDR -- enough that a real OpenOCD gets through `dap init` against it
  over the CMSIS-DAP TCP backend:

      openocd -c "adapter driver cmsis-dap" -c "cmsis-dap backend tcp" \
          -c "cmsis-dap tcp host 127.0.0.1" -c "cmsis-dap tcp port 4441" \
          -c "transport select swd" -c init

  It stops short of a MEM-AP ROM table and of a second core, so OpenOCD
  cannot go on to probe flash here, and useful for developing any
  CMSIS-DAP-over-TCP client end to end without a board. Round trips
  measured against it carry over to real hardware; the times do not, since
  nothing here is clocking an SWD wire.

`test_node_query.py`
: Drives `sim_probe` directly over the wire to check `CMD_NODE_QUERY` (0x87)
  and `CMD_NODE_DETECT` (0x92) -- the batched mux+connect(+RTT) vendor
  commands in `node_query.cpp`. Covers a detect-only query, a batch of
  several sub-requests landing in order in the down ring, request_count and
  sub-request-length bounds checking, a genuine write-now/poll-later round
  trip through the simulated target's echo, and the not-connected case.
  Starts its own `sim_probe` subprocess; run `make sim_probe` first.
