# cmsis_dap_tcp

A bitbanged SWD-to-CMSIS-DAP-over-TCP proxy for the Raspberry Pi (BCM283x /
BCM2711 GPIO, i.e. Pi 1-4 and CM1/CM3/CM4). It drives SWCLK/SWDIO directly
over `mmap`'ed GPIO registers and serves the same wire protocol OpenOCD's
`cmsis-dap tcp` backend speaks, so any CMSIS-DAP client can debug or flash a
target over the network instead of a local USB probe.

Adapted from [bkuschak/cmsis_dap_tcp_esp32](https://github.com/bkuschak/cmsis_dap_tcp_esp32/),
which serves the same protocol from an ESP32 over WiFi; that project in turn
took its CMSIS-DAP core from [ARM-software/CMSIS-DAP](https://github.com/ARM-software/CMSIS-DAP)
(v2.1.2). Here the ESP-IDF/FreeRTOS side is replaced by a plain Linux process
and the ESP32 GPIO driver by an mmap of the BCM283x/BCM2711 registers; the TCP
wire format is unchanged.

## What it does

- Speaks CMSIS-DAP (`DAP_Info`, `DAP_Transfer`, `DAP_TransferBlock`, ...) over
  a plain TCP socket, so `openocd -c "adapter driver cmsis-dap" -c "cmsis-dap
  backend tcp"` can point straight at it.
- Optional SWD mux support: an I2C GPIO expander selects which physical
  connector's SWD lines are routed to the shared bus, so one gateway can serve
  several target boards from one Pi. `--no-swd-mux` skips all of that for a
  single fixed target.
- SWD multi-drop bring-up (`dp_connect.h`): line reset, dormant-to-SWD switch,
  TARGETSEL selection and the DPIDR read ADIv5.2 requires before anything else
  answers -- the handshake a Cortex-M0+ multi-drop wire (e.g. an RP2040's two
  cores) needs and an easy step for a client to get wrong.
- SEGGER RTT support as CMSIS-DAP vendor commands (`rtt.h`): start/stop,
  status, buffered read/write against a target's RTT control block, without a
  full debugger attached.
- A fused mux-select + DP-connect(+RTT) vendor command (`node_query.h`) that
  folds an entire "is anything there, and if so run these requests" cycle into
  one round trip, for a client sweeping many mux positions where most of them
  are empty most of the time.
- RP2040 bootrom flashing as vendor commands (`rp2040.h`): attach, erase,
  program, verify and CRC, driven entirely from the probe side so a client
  only exchanges flash data and status, not individual bootrom calls.
- SWD clock self-calibration (`calibrate.h`): measure this Pi's own
  delay-loop and GPIO-write cost at startup instead of relying on fixed
  `speed_coeff`/`speed_offset` constants tuned for a different board.

## Building

Preferred: `./build_static_arm64.sh` cross-builds a fully static arm64
binary from any host, in an Alpine (musl) Docker container under QEMU
emulation -- no aarch64 cross-toolchain needed locally, and the result
drops onto a Pi with no shared libraries to matching versions of.

Otherwise, needs nothing but `g++` and `make` -- build directly on the Pi,
or cross-compile from anywhere with `aarch64-linux-gnu-g++`:

```bash
make                                    # ./cmsis_dap_tcp for the host
make -j$(nproc)                         # ... in parallel
make CXX=aarch64-linux-gnu-g++          # cross-compile
make static                             # fully static (needs static libstdc++/libc)
make clean                              # remove build/ and the binary
```

## Testing

```bash
make test        # rtt_test, shadow_test, rp2040_test -- run on any Linux host,
                  # no Pi, no SWD wire and no target board needed
make sim_probe    # a fake probe over TCP for developing a client against
```

See [tests/README.md](tests/README.md) for what each covers.

## Running

```bash
# listen on all interfaces, port 4441, select mux position 1 at startup
sudo ./cmsis_dap_tcp --bind 0.0.0.0 --port 4441 --swd-pos 1

# a board with different wiring and no SWD mux
sudo ./cmsis_dap_tcp --bind 127.0.0.1 --port 4441 --no-swd-mux \
    --swclk 16 --swdio-in 12 --swdio-out 20 --swdio-dir 21

# switch SWD mux position to 0x01 (node 1) via a raw ID_DAP_Vendor0 (0x80)
# command instead of --swd-pos -- useful to reselect without restarting
# cmsis_dap_tcp
openocd -c "adapter driver cmsis-dap" -c "cmsis-dap backend tcp" \
    -c "cmsis-dap tcp host 127.0.0.1" -c "cmsis-dap tcp port 4441" \
    -c "transport select swd" -c "init; cmsis-dap cmd 0x80 0x01; cmsis-dap info; shutdown"

# just check whether an SWD device answers on the currently selected
# position, without flashing or attaching -- e.g. "Info : SWD DPIDR
# 0x0bc12477, DLPIDR 0x00000001" means one did
openocd -c "adapter driver cmsis-dap" -c "cmsis-dap backend tcp" \
    -c "cmsis-dap tcp host 127.0.0.1" -c "cmsis-dap tcp port 4441" \
    -f target/rp2040.cfg -c "init; shutdown"

# flash firmware.bin via OpenOCD's own rp2040/rp2xxx flash driver --
# ordinary CMSIS-DAP transfers, none of this repo's own RP2040 flashing
# vendor commands needed
openocd -c "adapter driver cmsis-dap" -c "cmsis-dap backend tcp" \
    -c "cmsis-dap tcp host 127.0.0.1" -c "cmsis-dap tcp port 4441" \
    -f target/rp2040.cfg -c "program firmware.bin 0x10000000 verify reset exit"
```

Root (or GPIO-group membership) is needed for `/dev/gpiomem`/`/dev/mem`
access. Run with `--help` for the full flag list -- pin numbers, the I2C mux
bus/address, SWD clock calibration, `--unix-socket` for a same-machine client
that wants to skip the TCP/IP stack, and log level.

## Wire protocol

Standard CMSIS-DAP over the same length-prefixed framing
`cmsis_dap_tcp_esp32` and OpenOCD's `cmsis-dap backend tcp` use, plus a block
of vendor commands in the `ID_DAP_Vendor0..31` range. Multi-byte values are
little endian throughout; every response starts with the command ID echoed
back, then a one-byte status (table below), then the fields listed here --
on failure the fields after status are present but zero/meaningless unless
a command says otherwise. Full rationale and edge cases for each are in the
header named in its row.

### Status codes (`vendor.h`)

| Value | Name | Meaning |
|---|---|---|
| 0x00 | `STATUS_OK` | success |
| 0x01 | `STATUS_NOT_STARTED` | no `RTT_Start` has succeeded |
| 0x02 | `STATUS_NOT_CONNECTED` | debug port is not in SWD mode |
| 0x03 | `STATUS_TRANSFER` | a DP/AP transfer failed, or none answered |
| 0x04 | `STATUS_DAP_BUSY` | the DP had a sticky error on arrival |
| 0x05 | `STATUS_UNSUPPORTED` | the MEM-AP cannot do byte accesses |
| 0x06 | `STATUS_NOT_FOUND` | no control block at/in the given range |
| 0x07 | `STATUS_BAD_CHANNEL` | channel out of range |
| 0x08 | `STATUS_NO_BUFFER` | channel exists but the target left it unset |
| 0x09 | `STATUS_CORRUPT` | ring offsets outside the buffer |
| 0x0A | `STATUS_BAD_REQUEST` | malformed, truncated or oversized request |
| 0x0B | `STATUS_NO_POWER` | the target never acknowledged power-up |
| 0x0C | `STATUS_MUX_FAILED` | the SWD mux refused the switch or is disabled |
| 0x0D | `STATUS_NOT_HALTED` | the core had to be halted and is not |
| 0x0E | `STATUS_TIMEOUT` | the target did not get there in time |
| 0x0F | `STATUS_NO_BOOTROM` | no RP2040 bootrom at the expected address |
| 0x10 | `STATUS_NOT_ATTACHED` | no `RP_Attach` has succeeded |
| 0x11 | `STATUS_CALL_FAILED` | a ROM call halted away from the trampoline |
| 0x12 | `STATUS_NOT_MAPPED` | GPIO pins are not memory-mapped yet |
| 0x13 | `STATUS_RESCUE_FAILED` | the rescue DP did not release the reset |

### 0x80 `Mux_Select` (`swdmux.h`)

| | Fields |
|---|---|
| request | `u8 position` (0 = extender, 1..12 = node) |
| response | `u8 status` |

Switches which physical connector's SWD lines are routed to the shared bus,
via an I2C GPIO expander. `STATUS_MUX_FAILED` under `--no-swd-mux` or an
out-of-range position.

### 0x81-0x85 RTT (`rtt.h`)

A SEGGER RTT session moved onto the probe: normally the host reads/writes
the ring buffer's offsets and data itself, one CMSIS-DAP transfer at a
time -- expensive over TCP. These do a whole poll of a channel in one round
trip. They borrow a debug port the client has already brought up; they do
not initialize the DAP themselves.

| ID | Name | Request | Response |
|---|---|---|---|
| 0x81 | `RTT_Start` | `u8 ap, u32 address, u32 search_len` | `u8 status, u32 cb_addr, u8 num_up, u8 num_down` |
| 0x82 | `RTT_Stop` | - | `u8 status` |
| 0x83 | `RTT_Status` | `u8 direction (0=up,1=down), u8 channel` | `u8 status, u32 buffer, u32 size, u32 write_off, u32 read_off, u32 flags, u8 name_len, char name[name_len]` |
| 0x84 | `RTT_Read` | `u8 channel, u16 max_len` | `u8 status, u16 len, u8 data[len]` |
| 0x85 | `RTT_Write` | `u8 channel, u16 len, u8 data[len]` | `u8 status, u16 accepted` |

`RTT_Start` with `search_len == 0` treats `address` as the control block
itself; otherwise `[address, address+search_len)` is scanned for the RTT ID
string. `RTT_Read`/`RTT_Write` are partial by design: a read is capped at
what's left of the response packet, and a write's `accepted` may be less
than `len` if the ring is nearly full -- the caller retries the remainder.

### 0x86 `DP_Connect` / 0x93 `RP_Rescue` (`dp_connect.h`)

Debug port bring-up as one round trip instead of the dozen a host normally
spends on the switch sequence, TARGETSEL selection and power-up handshake.

| ID | Name | Request | Response |
|---|---|---|---|
| 0x86 | `DP_Connect` | `u8 flags, u32 targetsel` | `u8 status, u32 dpidr, u32 ctrl_stat` |
| 0x93 | `RP_Rescue` | `u8 flags, u32 targetsel` | `u8 status, u32 dpidr, u32 ctrl_stat` |

`DP_Connect` flags: bit 0 selects `targetsel` before reading DPIDR (needed
on an SWD multi-drop wire, e.g. an RP2040's two cores); bits 2:1 pick the
switch sequence (0 = try JTAG-to-SWD then dormant, 1 = force JTAG-to-SWD,
2 = force dormant). `RP_Rescue` resets an RP2040 into the bootrom via its
power-on state machine (instance 0xF), for a target whose flashed firmware
hangs or reconfigures pins the debug port can no longer reach through.

### 0x87 `Node_Query` / 0x92 `Node_Detect` (`node_query.h`)

Fuses mux-select + `DP_Connect` (+ RTT) into one round trip, for a client
sweeping many mux positions where most are empty most of the time:
`request_count == 0` (or `Node_Detect`, its smaller fixed-size equivalent)
is just "is it there", and a non-zero count also drains that many
already-framed sub-requests against it, one write-then-poll-read each.

| ID | Name | Request | Response |
|---|---|---|---|
| 0x87 | `Node_Query` | `u8 flags, u8 mux_pos, u32 targetsel, u8 ap, u32 cb_addr, u8 down_channel, u8 up_channel, u16 max_wait_ms, u8 request_count, (u16 len, u8 data[len]) * request_count` | `u8 status, u32 dpidr, u32 ctrl_stat, u8 response_count, (u16 write_accepted, u16 read_len, u8 read_data[read_len]) * response_count` |
| 0x92 | `Node_Detect` | `u8 flags, u8 mux_pos, u32 targetsel` | `u8 status, u32 dpidr, u32 ctrl_stat` |

`flags` bits 0/2:1 mean the same as `DP_Connect`'s; bit 3 skips the mux
select entirely (a bench rig with no mux wired up). `cb_addr` is used as-is
(no RAM search). Sub-requests/responses are matched by send order, not by
anything in whatever protocol they carry. A batch that doesn't all fit in
the response gets fewer completed sub-responses rather than one the client
would drop whole, and a genuine transport failure partway through stops the
batch early with `response_count` short of `request_count`.

### 0x88-0x90 RP2040 flashing (`rp2040.h`)

Programs flash by halting the core and running the bootrom's own QSPI
routines through its debug trampoline -- the same thing OpenOCD's
`rp2xxx.c` does -- but driven entirely from the probe side, so the client
exchanges flash data and a status rather than one round trip per register
write. A session is `RP_Attach` once, then `RP_Flash_Prep` /
`RP_Flash_Erase` / (`RP_Flash_Stage` + `RP_Flash_Program`, repeated) /
`RP_Flash_Finish` / optionally `RP_Flash_CRC`. Between Prep and Finish the
flash is not memory-mapped, so a session that dies in the middle leaves a
target that won't boot until something flashes it again -- the same
exposure OpenOCD has.

| ID | Name | Request | Response |
|---|---|---|---|
| 0x88 | `RP_Attach` | `u8 ap, u8 flags, u32 stack_top, u32 staging_addr, u32 staging_len` | `u8 status, u32 dhcsr, u32 rom_magic, u16 jump_table[8]` |
| 0x89 | `RP_Core` | `u8 action, u16 timeout_ms` | `u8 status, u32 dhcsr` |
| 0x8A | `RP_Call` | `u8 n_args, u16 func, u16 timeout_ms, u32 args[4]` | `u8 status, u32 r0` |
| 0x8B | `RP_Flash_Prep` | - | `u8 status` |
| 0x8C | `RP_Flash_Erase` | `u32 addr, u32 count, u32 block_size, u8 block_cmd, u16 timeout_ms` | `u8 status` |
| 0x8D | `RP_Flash_Stage` | `u32 offset, u16 len, u8 data[len]` | `u8 status, u16 written` |
| 0x8E | `RP_Flash_Program` | `u32 addr, u32 stage_off, u32 count, u16 timeout_ms` | `u8 status` |
| 0x8F | `RP_Flash_Finish` | `u8 flags` | `u8 status` |
| 0x90 | `RP_Flash_CRC` | `u32 addr, u32 count` | `u8 status, u32 crc32` |

`RP_Attach`'s `ap` picks which core's MEM-AP to use (0 or 1 -- either can
flash, since both see the same bootrom and SRAM); flags bit 0 halts the
core first, bit 1 resets and catches it halted out of reset instead
(implies bit 0, needed if what's running has reconfigured QSPI or is stuck
in a WFI). `RP_Flash_Stage` fills a staging buffer in target RAM a packet
at a time so `RP_Flash_Program` can write it in one ROM call instead of
one per packet; `addr`/`count` there are multiples of 256 (the ROM's page
size), and the client pads the tail of its last page with `0xFF` itself.
`RP_Flash_Finish` flags bit 0 also resumes the core. `RP_Flash_CRC` reads
back through the XIP window `RP_Flash_Finish` restored, and is capped at
`CRC_MAX_LEN` per call.

### 0x91 `Calibrate` (`calibrate.h`)

| | Fields |
|---|---|
| request | (no arguments) |
| response | `u8 status, u32 speed_coeff, u32 speed_offset` |

Re-measures this machine's delay-loop and GPIO-write cost -- in place of
the `--speed-coeff`/`--speed-offset` the process started with, which may
have been measured on a different Pi -- and applies the result
immediately. Drives real SWCLK edges (SWDIO left alone); safe with a
target already connected, but a session mid-transfer will see spurious
clocking while it runs.

## License

Apache License 2.0 -- see [LICENSE](LICENSE). Retains the original copyright
notices from the upstream projects this was adapted from (ARM Limited for the
CMSIS-DAP core, Brian Kuschak for the TCP transport it was ported from).
