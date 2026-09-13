// A fake probe: the real TCP framing (tcp_server.cpp) and the real RTT
#include <stdlib.h>
// commands (rtt.cpp, target_mem.cpp) over a simulated DP/MEM-AP and a
// simulated target holding an RTT control block. Lets the host-side client be
// exercised end to end without a Raspberry Pi or a target board.
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <string>

#include "DAP.h"
#include "logging.h"
#include "dp_connect.h"
#include "node_query.h"
#include "rp2040.h"
#include "rtt.h"
#include "swd_port.h"
#include "tcp_server.h"

// node_query.cpp calls into the real swdmux::select_pos(), which needs a
// real I2C mux; this test program stands in for it the same way
// tests/shadow_test.cpp does, always reporting whatever position was asked
// for as present -- absence in these tests comes from the simulated DP, not
// the mux.
namespace swdmux {
bool select_pos(unsigned int) { return true; }
bool is_enabled(void) { return true; }
int current_pos(void) { return -1; }
}  // namespace swdmux

// ---- simulated target (same model as rtt_test.cpp) ----
//
// Enough of an RP2040 to run the flash vendor commands against: the full 264kB
// of SRAM, a bootrom holding the function table those commands look up, an
// XIP-mapped flash array, and the ARMv6-M debug registers. What is *not* here
// is a CPU -- see core_resume_sim() for what stands in for one.
static const uint32_t RAM_BASE = 0x20000000, RAM_SIZE = 0x42000;
static uint8_t ram[RAM_SIZE];
static bool ram_mapped(uint32_t a) { return a >= RAM_BASE && a < RAM_BASE + RAM_SIZE; }

static const uint32_t ROM_BASE = 0x00000000, ROM_SIZE = 0x4000;
static uint8_t rom[ROM_SIZE];
static bool rom_mapped(uint32_t a) { return a < ROM_BASE + ROM_SIZE; }

static const uint32_t XIP_BASE = 0x10000000, FLASH_SIZE = 2u * 1024 * 1024;
static uint8_t flash[FLASH_SIZE];
static bool flash_mapped(uint32_t a) { return a >= XIP_BASE && a < XIP_BASE + FLASH_SIZE; }

// Bootrom entry points. The addresses are arbitrary -- nothing is decoded at
// them; exec_rom_func() switches on the one the trampoline was handed. The odd
// ones carry the Thumb bit a real table would have.
static const uint16_t ROM_TABLE            = 0x0100;
static const uint16_t FN_TRAMPOLINE        = 0x1000;
static const uint16_t FN_TRAMPOLINE_END    = 0x1002;
static const uint16_t FN_CONNECT_FLASH     = 0x2001;
static const uint16_t FN_EXIT_XIP          = 0x2011;
static const uint16_t FN_RANGE_ERASE       = 0x2021;
static const uint16_t FN_RANGE_PROGRAM     = 0x2031;
static const uint16_t FN_FLUSH_CACHE       = 0x2041;
static const uint16_t FN_ENTER_CMD_XIP     = 0x2051;

// ARMv6-M debug block. Word accesses to it have side effects, so it is handled
// whole rather than as bytes.
static const uint32_t PPB_BASE = 0xE000E000, PPB_SIZE = 0x1000;
static bool ppb_mapped(uint32_t a) { return a >= PPB_BASE && a < PPB_BASE + PPB_SIZE; }

struct Core {
  // r0..r15, then xPSR at 16 and MSP at 17, as DCRSR numbers them.
  uint32_t r[18] = {0};
  uint32_t dcrdr = 0, demcr = 0;
  bool halted = false;      // S_HALT
  bool debugen = false;     // C_DEBUGEN
  bool maskints = false;    // C_MASKINTS
  bool reset_st = true;     // S_RESET_ST, sticky and cleared by a read
  bool lockup = false;      // S_LOCKUP
} core;

// Whether the flash is memory mapped. flash_exit_xip takes it out of the map
// and flash_enter_cmd_xip puts it back, so a client that reads the XIP window
// between the two gets the fault a real one would get garbage for.
static bool xip_enabled = true;

struct Dp {
  uint32_t select = 0, ctrl_stat = 0x00000040, posted = 0;
  uint32_t csw = 0x23000052, tar = 0;
} sim_dp;

// SWD multi-drop, as an RP2040 has it: after a line reset no DP answers until
// one is picked out by a TARGETSEL write. Off unless asked for on the command
// line, so the plain single-DP case stays testable too.
static bool     multidrop = false;
static uint32_t targetsel_expect = 0x01002927;   // RP2040 core 0
static bool     dp_selected = true;
// ADIv5.2: the DP a TARGETSEL write picks answers nothing until the host reads
// DPIDR, which is how it knows the selection took. A host that goes straight to
// some other register gets no acknowledge at all -- modelled here because it is
// an easy step to leave out and an obscure failure to debug without it.
static bool     awaiting_dpidr = false;

// A target that never acknowledges the power-up request, to exercise the
// bring-up's timeout.
static bool     no_power = false;

// The one thing standing in for a CPU. The only code these tests ever set
// running is the bootrom's debug trampoline, whose whole body is `blx r7`
// followed by a breakpoint -- so "resume" means "carry out the routine in r7,
// then halt at the breakpoint", with no instruction decoding in between.
static bool exec_rom_func(uint32_t func, const uint32_t *arg);

static void core_resume_sim() {
  core.lockup = false;        // halting is how a core leaves lockup
  if ((core.r[15] & ~1u) != FN_TRAMPOLINE) {
    core.halted = false;      // application code: nothing here models it
    return;
  }
  if (exec_rom_func(core.r[7], core.r)) {
    core.r[15] = FN_TRAMPOLINE_END;   // the trampoline's own bkpt
  } else {
    core.r[15] = 0xFFFFFFFE;          // the lockup address
    core.lockup = true;
  }
  core.halted = true;
}

static uint32_t dhcsr_read() {
  uint32_t v = 0;
  if (core.debugen)  v |= 1u << 0;
  if (core.halted)   v |= 1u << 1;
  if (core.maskints) v |= 1u << 3;
  v |= 1u << 16;                                  // S_REGRDY: always settled
  if (core.halted)   v |= 1u << 17;
  if (core.lockup)   v |= 1u << 19;
  if (core.reset_st) { v |= 1u << 25; core.reset_st = false; }   // clears on read
  return v;
}

static void dhcsr_write(uint32_t v) {
  if ((v >> 16) != 0xA05Fu) return;               // no key, no write
  const bool was_halted = core.halted;
  core.debugen  = v & (1u << 0);
  core.maskints = v & (1u << 3);
  if (v & (1u << 1)) {
    core.halted = true;
  } else if (was_halted && core.debugen) {
    core_resume_sim();
  }
}

static void dcrsr_write(uint32_t v) {
  const unsigned sel = v & 0x1Fu;
  if (sel >= 18) return;
  if (v & (1u << 16)) core.r[sel] = core.dcrdr;
  else                core.dcrdr = core.r[sel];
}

static void aircr_write(uint32_t v) {
  if ((v >> 16) != 0x05FAu) return;
  if (!(v & (1u << 2))) return;                   // not SYSRESETREQ
  core.reset_st = true;
  core.lockup = false;
  core.r[15] = 0x000000EAu;                       // wherever the bootrom starts
  core.halted = core.demcr & 1u;                  // VC_CORERESET
  xip_enabled = true;                             // a reset re-runs the boot path
}

// Word accesses to the debug block, which is where the side effects live.
static uint32_t ppb_read(uint32_t addr) {
  switch (addr) {
    case 0xE000EDF0: return dhcsr_read();
    case 0xE000EDF8: return core.dcrdr;
    case 0xE000EDFC: return core.demcr;
    case 0xE000ED0C: return 0xFA050000u;
    default: return 0;
  }
}

static void ppb_write(uint32_t addr, uint32_t v) {
  switch (addr) {
    case 0xE000EDF0: dhcsr_write(v); break;
    case 0xE000EDF4: dcrsr_write(v); break;
    case 0xE000EDF8: core.dcrdr = v; break;
    case 0xE000EDFC: core.demcr = v; break;
    case 0xE000ED0C: aircr_write(v); break;
    default: break;
  }
}

// Byte-level view of everything that is not the debug block. Returns false for
// an address nothing answers at, which the caller turns into a sticky error.
static bool mem_byte_read(uint32_t a, uint8_t *out) {
  if (ram_mapped(a))   { *out = ram[a - RAM_BASE];  return true; }
  if (rom_mapped(a))   { *out = rom[a - ROM_BASE];  return true; }
  if (flash_mapped(a)) {
    if (!xip_enabled) return false;                // not in the map right now
    *out = flash[a - XIP_BASE];
    return true;
  }
  return false;
}

static bool mem_byte_write(uint32_t a, uint8_t v) {
  if (ram_mapped(a)) { ram[a - RAM_BASE] = v; return true; }
  return false;                                    // ROM and XIP are read only
}

static uint32_t mem_read_sized(uint32_t addr, unsigned size) {
  if (ppb_mapped(addr)) {
    if (size != 2 || (addr & 3u)) { sim_dp.ctrl_stat |= swd::CTRL_STICKYERR; return 0; }
    return ppb_read(addr);
  }
  uint32_t v = 0; unsigned n = 1u << size;
  for (unsigned i = 0; i < n; i++) {
    uint8_t b;
    if (!mem_byte_read(addr + i, &b)) { sim_dp.ctrl_stat |= swd::CTRL_STICKYERR; return 0; }
    v |= (uint32_t)b << (8 * i);
  }
  return size == 0 ? (v << (8 * (addr & 3u))) : v;
}

static void mem_write_sized(uint32_t addr, unsigned size, uint32_t v) {
  if (ppb_mapped(addr)) {
    if (size != 2 || (addr & 3u)) sim_dp.ctrl_stat |= swd::CTRL_STICKYERR;
    else                          ppb_write(addr, v);
    return;
  }
  unsigned n = 1u << size;
  if (size == 0) v >>= 8 * (addr & 3u);
  for (unsigned i = 0; i < n; i++) {
    if (!mem_byte_write(addr + i, (uint8_t)(v >> (8 * i)))) {
      sim_dp.ctrl_stat |= swd::CTRL_STICKYERR;
      return;
    }
  }
}

static void tar_advance(unsigned size) {
  unsigned inc = 0;
  switch ((sim_dp.csw & swd::CSW_ADDRINC_MASK) >> 4) {
    case 1: inc = 1u << size; break;
    case 2: inc = 4; break;
    default: return;
  }
  sim_dp.tar = (sim_dp.tar & ~1023u) | ((sim_dp.tar + inc) & 1023u);
}

namespace swd {
uint8_t transfer(uint32_t request, uint32_t *data) {
  if (!dp_selected) {
    return 7;                 // nothing drives the ack: the line reads high
  }
  const uint32_t reg = request & REQ_ADDR;
  const bool is_ap = request & REQ_APnDP, is_read = request & REQ_RnW;
  if (awaiting_dpidr) {
    if (is_ap || !is_read || reg != 0x00) {
      return 7;               // not the DPIDR read the selection is waiting for
    }
    awaiting_dpidr = false;
  }
  if (!is_ap) {
    if (is_read) {
      switch (reg) {
        case DP_REG_CTRL_STAT:
          // Address 0x4 is CTRL/STAT in DP bank 0 and DLPIDR in bank 3. A
          // multi-drop host reads DLPIDR to confirm the TARGETSEL write picked
          // the instance it asked for, so the banked view has to exist or the
          // selection looks like it silently failed.
          if (data) {
            *data = ((sim_dp.select & 0xFU) == 3U)
                        ? ((targetsel_expect & 0xF0000000U) | 0x1U)  // DLPIDR
                        : sim_dp.ctrl_stat;
          }
          return ACK_OK;
        case DP_REG_RDBUFF:    if (data) *data = sim_dp.posted;    return ACK_OK;
        case 0x00:
          // DPIDR. Multi-drop is a DPv2 feature, so a wire modelling it has to
          // answer with a DPv2 part or a host will refuse to use TARGETSEL at
          // all: 0x0BC12477 is the RP2040's, version 2 in bits 15:12.
          if (data) *data = multidrop ? 0x0BC12477U : 0x2BA01477U;
          return ACK_OK;
        default: return 4;
      }
    }
    switch (reg) {
      case DP_REG_SELECT: sim_dp.select = *data; return ACK_OK;
      case DP_REG_CTRL_STAT:
        // Grant debug and system power by echoing each request bit back one
        // place up, as the acknowledge bit sitting above it -- unless we were
        // asked to model a target that never grants it.
        sim_dp.ctrl_stat = (*data & 0x50000000u) |
                           (no_power ? 0u : ((*data & 0x50000000u) << 1)) |
                           (sim_dp.ctrl_stat & 0x000000FFu);
        return ACK_OK;
      case DP_REG_ABORT:
        if (*data & ABORT_STKERRCLR)  sim_dp.ctrl_stat &= ~CTRL_STICKYERR;
        if (*data & ABORT_STKCMPCLR)  sim_dp.ctrl_stat &= ~CTRL_STICKYCMP;
        if (*data & ABORT_WDERRCLR)   sim_dp.ctrl_stat &= ~CTRL_WDATAERR;
        if (*data & ABORT_ORUNERRCLR) sim_dp.ctrl_stat &= ~CTRL_STICKYORUN;
        return ACK_OK;
      default: return 4;
    }
  }
  if (sim_dp.ctrl_stat & CTRL_STICKY_ANY) return 4;
  if (((sim_dp.select >> 4) & 0xF) != 0) return 4;
  const unsigned size = sim_dp.csw & CSW_SIZE_MASK;
  if (is_read) {
    if (data) *data = sim_dp.posted;
    switch (reg) {
      case AP_REG_CSW: sim_dp.posted = sim_dp.csw; break;
      case AP_REG_TAR: sim_dp.posted = sim_dp.tar; break;
      case AP_REG_DRW: sim_dp.posted = mem_read_sized(sim_dp.tar, size); tar_advance(size); break;
      default: return 4;
    }
    return ACK_OK;
  }
  switch (reg) {
    case AP_REG_CSW: sim_dp.csw = *data; break;
    case AP_REG_TAR: sim_dp.tar = *data; break;
    case AP_REG_DRW: mem_write_sized(sim_dp.tar, size, *data); tar_advance(size); break;
    default: return 4;
  }
  return ACK_OK;
}
// Sequence-level modelling of a multi-drop wire, shared by the raw
// DAP_SWJ_Sequence / DAP_SWD_Sequence commands and by the probe-side
// bring-up in dp_connect.cpp, so both drive the same fake DP.
void sequence_swj(uint32_t bits, const uint8_t *data) {
  uint32_t run = 0, longest = 0;
  for (uint32_t i = 0; i < bits; i++) {
    if ((data[i / 8] >> (i % 8)) & 1U) {
      if (++run > longest) longest = run;
    } else {
      run = 0;
    }
  }
  // 50 or more clocks with the line high is a reset, and a reset leaves every
  // DP on a multi-drop wire deselected.
  if (multidrop && longest >= 50) dp_selected = false;
}

void sequence_swd(uint8_t info, const uint8_t *out, uint8_t *in) {
  static bool expect_value = false;
  uint32_t bits = info & 0x3F;
  if (bits == 0) bits = 64;
  const uint32_t bytes = (bits + 7U) / 8U;

  if (info & SEQ_CAPTURE) {
    if (in) memset(in, 0, bytes);   // nothing drives these back
    return;
  }
  if (bits == 8 && out[0] == 0x99) {
    expect_value = true;            // DP write to TARGETSEL
    return;
  }
  if (expect_value && bits == 33) {
    uint32_t value;
    memcpy(&value, out, 4);
    if (multidrop && value == targetsel_expect) {
      dp_selected = true;
      awaiting_dpidr = true;
    }
    expect_value = false;
  }
}

static Shadow g_shadow;
const Shadow &shadow() { return g_shadow; }
void snoop_suspend() {}
void snoop_resume() {}
void shadow_reset() {}
bool port_is_swd() { return true; }
}  // namespace swd

// ---- RTT control block in the simulated target ----
static const uint32_t CB = 0x20001000, UP_BUF = 0x20002000, UP_SIZE = 128;
static const uint32_t DN_BUF = 0x20003000, DN_SIZE = 64;
static const uint32_t NAME = 0x20001200;

static void poke32(uint32_t a, uint32_t v) { for (int i=0;i<4;i++) ram[a-RAM_BASE+i]=(uint8_t)(v>>(8*i)); }
static uint32_t peek32(uint32_t a) { uint32_t v=0; for (int i=0;i<4;i++) v|=(uint32_t)ram[a-RAM_BASE+i]<<(8*i); return v; }

static void build_cb() {
  memcpy(ram + (CB - RAM_BASE), "SEGGER RTT", 11);
  poke32(CB + 16, 1);
  poke32(CB + 20, 1);
  memcpy(ram + (NAME - RAM_BASE), "Terminal", 9);
  poke32(CB + 24 +  0, NAME);
  poke32(CB + 24 +  4, UP_BUF);
  poke32(CB + 24 +  8, UP_SIZE);
  poke32(CB + 48 +  0, NAME);
  poke32(CB + 48 +  4, DN_BUF);
  poke32(CB + 48 +  8, DN_SIZE);
}

// The bootrom function table the RP2040 flash commands look up: the magic
// word at 0x10, a u16 pointer to the table at 0x14, and then tag/address
// pairs terminated by a zero tag.
static void build_rom() {
  memset(rom, 0, sizeof(rom));
  rom[0x10] = 0x4D; rom[0x11] = 0x75; rom[0x12] = 0x01;   // 'M' 'u' version 1
  rom[0x14] = (uint8_t)ROM_TABLE; rom[0x15] = (uint8_t)(ROM_TABLE >> 8);

  static const struct { char a, b; uint16_t addr; } entries[] = {
    {'D', 'T', FN_TRAMPOLINE},   {'D', 'E', FN_TRAMPOLINE_END},
    {'I', 'F', FN_CONNECT_FLASH}, {'E', 'X', FN_EXIT_XIP},
    {'R', 'E', FN_RANGE_ERASE},  {'R', 'P', FN_RANGE_PROGRAM},
    {'F', 'C', FN_FLUSH_CACHE},  {'C', 'X', FN_ENTER_CMD_XIP},
  };
  uint8_t *p = rom + ROM_TABLE;
  for (const auto &e : entries) {
    *p++ = (uint8_t)e.a; *p++ = (uint8_t)e.b;
    *p++ = (uint8_t)e.addr; *p++ = (uint8_t)(e.addr >> 8);
  }
  memset(p, 0, 4);                                  // terminator
}

// What the bootrom routines do to the simulated flash. Arguments arrive in
// r0..r3 exactly as the trampoline left them. False for a routine that is not
// in the table: the trampoline branched into nothing.
static bool exec_rom_func(uint32_t func, const uint32_t *arg) {
  switch (func) {
    case FN_CONNECT_FLASH:
    case FN_FLUSH_CACHE:
      break;                                        // nothing to model
    case FN_EXIT_XIP:
      xip_enabled = false;
      break;
    case FN_ENTER_CMD_XIP:
      xip_enabled = true;
      break;
    case FN_RANGE_ERASE: {
      const uint32_t off = arg[0], count = arg[1];
      if ((uint64_t)off + count <= FLASH_SIZE) memset(flash + off, 0xFF, count);
      break;
    }
    case FN_RANGE_PROGRAM: {
      const uint32_t off = arg[0], src = arg[1], count = arg[2];
      if ((uint64_t)off + count > FLASH_SIZE) break;
      for (uint32_t i = 0; i < count; i++) {
        uint8_t b = 0xFF;
        (void)mem_byte_read(src + i, &b);
        // Programming can only clear bits, which is what makes an erase
        // necessary in the first place; a client that skips one should see it.
        flash[off + i] &= b;
      }
      break;
    }
    default:
      return false;
  }
  return true;
}

static void target_emit(const std::string &s) {
  uint32_t wr = peek32(CB + 24 + 12), rd = peek32(CB + 24 + 16);
  for (char c : s) {
    if ((wr + 1) % UP_SIZE == rd) break;
    ram[UP_BUF - RAM_BASE + wr] = (uint8_t)c;
    wr = (wr + 1) % UP_SIZE;
  }
  poke32(CB + 24 + 12, wr);
}

// The target echoes what it is sent, in upper case.
static void target_poll() {
  uint32_t wr = peek32(CB + 48 + 12), rd = peek32(CB + 48 + 16);
  std::string got;
  while (rd != wr) { got += (char)ram[DN_BUF - RAM_BASE + rd]; rd = (rd + 1) % DN_SIZE; }
  poke32(CB + 48 + 16, rd);
  if (!got.empty()) {
    for (char &c : got) if (c >= 'a' && c <= 'z') c -= 32;
    target_emit("echo: " + got);
  }
}

// An AP read on the wire returns the *previous* access's result and starts a
// new one, so a CMSIS-DAP implementation collects the last one from DP RDBUFF
// before handing anything back. swd::transfer() models the raw wire, so the
// command handlers below have to do that flush themselves -- otherwise every
// AP read a client makes comes back one access stale.
static uint8_t ap_read_flushed(uint32_t req, uint32_t *value) {
  uint32_t stale = 0;
  const uint8_t ack = swd::transfer(req, &stale);
  if (ack != 1) return ack;
  return swd::transfer(swd::REQ_RnW | swd::DP_REG_RDBUFF, value);
}

// ---- the DAP command surface the client uses ----
namespace cmsis { namespace dap {

uint32_t process_cmd(const uint8_t *request, uint8_t *response) {
  target_poll();      // the simulated CPU runs between commands

  const uint8_t cmd = request[0];
  if (rtt::handles(cmd)) {
    return rtt::handle_command(request, response, MAX_PACKET_SIZE,
                                MAX_PACKET_SIZE);
  }
  if (node_query::handles(cmd)) {
    return node_query::handle_command(request, response, MAX_PACKET_SIZE,
                                      MAX_PACKET_SIZE);
  }
  if (rp2040::handles(cmd)) {
    return rp2040::handle_command(request, response, MAX_PACKET_SIZE);
  }
  // Asking the module which commands are its own, rather than listing them
  // here: a command added to dp_connect.cpp is then reachable without this
  // file having to learn about it.
  if (dp::handles(cmd)) {
    return dp::handle_command(request, response, MAX_PACKET_SIZE);
  }
  switch (cmd) {
    case 0x00: {   // DAP_Info -- every real client asks for the packet size
      response[0] = 0x00;
      switch (request[1]) {
        case 0xFF:   // max packet size
          response[1] = 2;
          response[2] = (uint8_t)MAX_PACKET_SIZE;
          response[3] = (uint8_t)(MAX_PACKET_SIZE >> 8);
          return (2U << 16) | 4U;
        case 0xFE:   // max packet count
          response[1] = 1; response[2] = 1;
          return (2U << 16) | 3U;
        case 0xF0:   // capabilities: SWD only, which is all this bitbangs
          response[1] = 1; response[2] = 0x01;
          return (2U << 16) | 3U;
        default:     // the string IDs: present but empty
          response[1] = 0;
          return (2U << 16) | 2U;
      }
    }
    case 0x06: {   // DAP_TransferBlock
      const uint16_t count = (uint16_t)(request[2] | (request[3] << 8));
      const uint8_t  req   = request[4];
      const bool     is_read = (req & 0x02) != 0;
      const uint8_t *in  = request + 5;
      uint8_t       *out = response + 4;
      uint16_t done = 0;
      uint8_t  ack  = 1;
      if (is_read && (req & 0x01)) {
        // Pipelined AP reads: the k-th one returns what the (k-1)-th started,
        // and RDBUFF at the end returns the last. That is one AP access per
        // word, which is what keeps TAR's auto-increment in step.
        uint32_t value = 0;
        for (uint16_t k = 0; k < count; k++) {
          ack = swd::transfer(req, &value);
          if (ack != 1) break;
          if (k >= 1) { memcpy(out, &value, 4); out += 4; done++; }
        }
        if (ack == 1) {
          ack = swd::transfer(swd::REQ_RnW | swd::DP_REG_RDBUFF, &value);
          if (ack == 1) { memcpy(out, &value, 4); out += 4; done++; }
        }
      } else {
        for (; done < count; done++) {
          uint32_t value = 0;
          if (!is_read) { memcpy(&value, in, 4); in += 4; }
          ack = swd::transfer(req, &value);
          if (ack != 1) break;
          if (is_read) { memcpy(out, &value, 4); out += 4; }
        }
      }
      response[0] = 0x06;
      response[1] = (uint8_t)done;
      response[2] = (uint8_t)(done >> 8);
      response[3] = ack;
      const uint32_t taken = 5U + (is_read ? 0U : (uint32_t)count * 4U);
      return (taken << 16) | (uint32_t)(out - response);
    }
    case 0x02: response[0] = 0x02; response[1] = 0x01; return (2U << 16) | 2U;
    case 0x04: response[0] = 0x04; response[1] = 0x00; return (6U << 16) | 2U;
    case 0x11: response[0] = 0x11; response[1] = 0x00; return (5U << 16) | 2U;
    case 0x13: response[0] = 0x13; response[1] = 0x00; return (2U << 16) | 2U;
    case 0x12: {   // DAP_SWJ_Sequence
      const uint32_t bits = request[1] ? request[1] : 256U;
      swd::sequence_swj(bits, request + 2);
      response[0] = 0x12; response[1] = 0x00;
      return (((2U + (bits + 7U) / 8U) << 16) | 2U);
    }
    case 0x1D: {   // DAP_SWD_Sequence
      const uint8_t entries = request[1];
      const uint8_t *p = request + 2;
      uint8_t *out = response + 2;
      for (uint8_t i = 0; i < entries; i++) {
        const uint8_t info = *p++;
        uint32_t bits = info & 0x3F;
        if (bits == 0) bits = 64;
        const uint32_t bytes = (bits + 7U) / 8U;
        if (info & 0x80) {
          swd::sequence_swd(info, nullptr, out);
          out += bytes;
        } else {
          swd::sequence_swd(info, p, nullptr);
          p += bytes;
        }
      }
      response[0] = 0x1D; response[1] = 0x00;
      return (((uint32_t)(p - request) << 16) | (uint32_t)(out - response));
    }
    case 0x05: {
      const uint8_t count = request[2];
      const uint8_t *p = request + 3;
      uint8_t *out = response + 3;
      uint8_t done = 0, ack = 1;
      for (uint8_t i = 0; i < count; i++) {
        const uint8_t req = *p++;
        uint32_t value = 0;
        if (!(req & 0x02)) { memcpy(&value, p, 4); p += 4; }
        ack = ((req & 0x03) == 0x03) ? ap_read_flushed(req, &value)
                                     : swd::transfer(req, &value);
        if (ack != 1) break;
        if (req & 0x02) { memcpy(out, &value, 4); out += 4; }
        done++;
      }
      response[0] = 0x05; response[1] = done; response[2] = ack;
      return (((uint32_t)(p - request) << 16) | (uint32_t)(out - response));
    }
    default:
      response[0] = 0xFF;
      return (1U << 16) | 1U;
  }
}

uint32_t execute_cmd(const uint8_t *request, uint8_t *response) {
  return process_cmd(request, response);
}

}}  // namespace cmsis::dap

int main(int argc, char **argv) {
  logging::init(logging::Level::warn);
  const char *unix_socket = NULL;
  for (int i = 2; i < argc; i++) {
    if (strcmp(argv[i], "--multidrop") == 0) {
      multidrop = true;
      dp_selected = false;
    } else if (strcmp(argv[i], "--nopower") == 0) {
      no_power = true;
    } else if (strncmp(argv[i], "--targetsel=", 12) == 0) {
      targetsel_expect = (uint32_t)strtoul(argv[i] + 12, NULL, 0);
    } else if (strncmp(argv[i], "--unix-socket=", 14) == 0) {
      unix_socket = argv[i] + 14;
    }
  }
  build_rom();
  memset(flash, 0xFF, sizeof(flash));
  build_cb();
  target_emit("boot: simulated target ready\n");
  tcp_server::install_exit_handlers();
  tcp_server::serve("127.0.0.1", argc > 1 ? atoi(argv[1]) : 4441, unix_socket);
  return 0;
}
