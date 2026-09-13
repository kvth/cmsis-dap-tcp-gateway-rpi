// Test harness for the RP2040 flash vendor commands: a fake SWD DP/MEM-AP over
// a simulated RP2040 -- SRAM, a bootrom holding the function table, an
// XIP-mapped flash array and the ARMv6-M debug registers -- driving the real
// rp2040.cpp and target_mem.cpp.
//
// There is no CPU here. The only code these commands ever set running is the
// bootrom's debug trampoline, whose body is `blx r7` followed by a breakpoint,
// so "resume" means "carry out the routine named in r7, then halt".
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>

#include "rp2040.h"
#include "swd_port.h"
#include "vendor.h"

// ---------------- simulated target ----------------
static const uint32_t RAM_BASE = 0x20000000, RAM_SIZE = 0x42000;
static uint8_t ram[RAM_SIZE];

static const uint32_t ROM_SIZE = 0x4000;
static uint8_t rom[ROM_SIZE];

static const uint32_t XIP_BASE = 0x10000000, FLASH_SIZE = 512 * 1024;
static uint8_t flash[FLASH_SIZE];

static const uint16_t ROM_TABLE         = 0x0100;
static const uint16_t FN_TRAMPOLINE     = 0x1000;
static const uint16_t FN_TRAMPOLINE_END = 0x1002;
static const uint16_t FN_CONNECT_FLASH  = 0x2001;
static const uint16_t FN_EXIT_XIP       = 0x2011;
static const uint16_t FN_RANGE_ERASE    = 0x2021;
static const uint16_t FN_RANGE_PROGRAM  = 0x2031;
static const uint16_t FN_FLUSH_CACHE    = 0x2041;
static const uint16_t FN_ENTER_CMD_XIP  = 0x2051;

struct Dp {
  uint32_t select = 0, ctrl_stat = 0xF0000040, posted = 0;
  uint32_t csw = 0x23000052, tar = 0;
} dp;

struct Core {
  uint32_t r[18] = {0};             // r0..r15, xPSR at 16, MSP at 17
  uint32_t dcrdr = 0, demcr = 0;
  bool halted = false, debugen = false, maskints = false;
  bool reset_st = true, lockup = false;
} core;

static bool xip_enabled = true;
static unsigned rom_calls = 0;

// False for a routine that is not in the table: the trampoline branched into
// nothing and the core never gets back to the breakpoint.
static bool exec_rom_func(uint32_t func, const uint32_t *arg) {
  rom_calls++;
  switch (func) {
    case FN_CONNECT_FLASH:
    case FN_FLUSH_CACHE:
      break;
    case FN_EXIT_XIP:      xip_enabled = false; break;
    case FN_ENTER_CMD_XIP: xip_enabled = true;  break;
    case FN_RANGE_ERASE: {
      const uint32_t off = arg[0], count = arg[1];
      if ((uint64_t)off + count <= FLASH_SIZE) memset(flash + off, 0xFF, count);
      break;
    }
    case FN_RANGE_PROGRAM: {
      const uint32_t off = arg[0], src = arg[1], count = arg[2];
      if ((uint64_t)off + count > FLASH_SIZE) break;
      for (uint32_t i = 0; i < count; i++) {
        const uint32_t a = src + i;
        // Programming only clears bits, which is what makes the erase matter.
        const uint8_t b = (a >= RAM_BASE && a < RAM_BASE + RAM_SIZE)
                              ? ram[a - RAM_BASE] : 0xFF;
        flash[off + i] &= b;
      }
      break;
    }
    default:
      return false;
  }
  return true;
}

static void core_resume_sim() {
  core.lockup = false;              // halting is how a core leaves lockup
  if ((core.r[15] & ~1u) != FN_TRAMPOLINE) { core.halted = false; return; }
  if (exec_rom_func(core.r[7], core.r)) {
    core.r[15] = FN_TRAMPOLINE_END; // the trampoline's own bkpt
  } else {
    core.r[15] = 0xFFFFFFFE;        // the lockup address
    core.lockup = true;
  }
  core.halted = true;
}

static uint32_t dhcsr_read() {
  uint32_t v = 1u << 16;                                  // S_REGRDY
  if (core.debugen)  v |= 1u << 0;
  if (core.halted)   v |= (1u << 1) | (1u << 17);
  if (core.maskints) v |= 1u << 3;
  if (core.lockup)   v |= 1u << 19;
  if (core.reset_st) { v |= 1u << 25; core.reset_st = false; }
  return v;
}

static void ppb_write(uint32_t addr, uint32_t v) {
  switch (addr) {
    case 0xE000EDF0: {                                    // DHCSR
      if ((v >> 16) != 0xA05Fu) return;
      const bool was_halted = core.halted;
      core.debugen  = v & (1u << 0);
      core.maskints = v & (1u << 3);
      if (v & (1u << 1))                    core.halted = true;
      else if (was_halted && core.debugen)  core_resume_sim();
      break;
    }
    case 0xE000EDF4: {                                    // DCRSR
      const unsigned sel = v & 0x1Fu;
      if (sel >= 18) break;
      if (v & (1u << 16)) core.r[sel] = core.dcrdr;
      else                core.dcrdr = core.r[sel];
      break;
    }
    case 0xE000EDF8: core.dcrdr = v; break;
    case 0xE000EDFC: core.demcr = v; break;
    case 0xE000ED0C:                                      // AIRCR
      if ((v >> 16) == 0x05FAu && (v & (1u << 2))) {
        core.reset_st = true;
        core.lockup = false;
        core.r[15] = 0xEA;
        core.halted = core.demcr & 1u;                    // VC_CORERESET
        xip_enabled = true;
      }
      break;
    default: break;
  }
}

static uint32_t ppb_read(uint32_t addr) {
  switch (addr) {
    case 0xE000EDF0: return dhcsr_read();
    case 0xE000EDF8: return core.dcrdr;
    case 0xE000EDFC: return core.demcr;
    default:         return 0;
  }
}

static bool ppb_mapped(uint32_t a) { return a >= 0xE000E000 && a < 0xE000F000; }

static bool mem_byte_read(uint32_t a, uint8_t *out) {
  if (a >= RAM_BASE && a < RAM_BASE + RAM_SIZE) { *out = ram[a - RAM_BASE]; return true; }
  if (a < ROM_SIZE)                             { *out = rom[a]; return true; }
  if (a >= XIP_BASE && a < XIP_BASE + FLASH_SIZE) {
    if (!xip_enabled) return false;             // not in the map right now
    *out = flash[a - XIP_BASE];
    return true;
  }
  return false;
}

static uint32_t mem_read_sized(uint32_t addr, unsigned size) {
  if (ppb_mapped(addr)) {
    if (size != 2 || (addr & 3u)) { dp.ctrl_stat |= swd::CTRL_STICKYERR; return 0; }
    return ppb_read(addr);
  }
  uint32_t v = 0; unsigned n = 1u << size;
  for (unsigned i = 0; i < n; i++) {
    uint8_t b;
    if (!mem_byte_read(addr + i, &b)) { dp.ctrl_stat |= swd::CTRL_STICKYERR; return 0; }
    v |= (uint32_t)b << (8 * i);
  }
  return size == 0 ? (v << (8 * (addr & 3u))) : v;
}

static void mem_write_sized(uint32_t addr, unsigned size, uint32_t v) {
  if (ppb_mapped(addr)) {
    if (size != 2 || (addr & 3u)) dp.ctrl_stat |= swd::CTRL_STICKYERR;
    else                          ppb_write(addr, v);
    return;
  }
  unsigned n = 1u << size;
  if (size == 0) v >>= 8 * (addr & 3u);
  for (unsigned i = 0; i < n; i++) {
    const uint32_t a = addr + i;
    if (a < RAM_BASE || a >= RAM_BASE + RAM_SIZE) {   // ROM and XIP are read only
      dp.ctrl_stat |= swd::CTRL_STICKYERR;
      return;
    }
    ram[a - RAM_BASE] = (uint8_t)(v >> (8 * i));
  }
}

static void tar_advance(unsigned size) {
  unsigned inc = 0;
  switch ((dp.csw & swd::CSW_ADDRINC_MASK) >> 4) {
    case 1: inc = 1u << size; break;
    case 2: inc = 4; break;
    default: return;
  }
  dp.tar = (dp.tar & ~1023u) | ((dp.tar + inc) & 1023u);
}

namespace swd {

uint8_t transfer(uint32_t request, uint32_t *data) {
  const uint32_t reg = request & REQ_ADDR;
  const bool is_ap = request & REQ_APnDP, is_read = request & REQ_RnW;
  if (!is_ap) {
    if (is_read) {
      switch (reg) {
        case DP_REG_CTRL_STAT: if (data) *data = dp.ctrl_stat; return ACK_OK;
        case DP_REG_RDBUFF:    if (data) *data = dp.posted;    return ACK_OK;
        case 0x00:             if (data) *data = 0x0BC12477;   return ACK_OK;
        default: return 4;
      }
    }
    switch (reg) {
      case DP_REG_SELECT: dp.select = *data; return ACK_OK;
      case DP_REG_ABORT:
        if (*data & ABORT_STKERRCLR)  dp.ctrl_stat &= ~CTRL_STICKYERR;
        if (*data & ABORT_STKCMPCLR)  dp.ctrl_stat &= ~CTRL_STICKYCMP;
        if (*data & ABORT_WDERRCLR)   dp.ctrl_stat &= ~CTRL_WDATAERR;
        if (*data & ABORT_ORUNERRCLR) dp.ctrl_stat &= ~CTRL_STICKYORUN;
        return ACK_OK;
      default: return 4;
    }
  }
  if (dp.ctrl_stat & CTRL_STICKY_ANY) return 4;
  const unsigned size = dp.csw & CSW_SIZE_MASK;
  if (is_read) {
    if (data) *data = dp.posted;
    switch (reg) {
      case AP_REG_CSW: dp.posted = dp.csw; break;
      case AP_REG_TAR: dp.posted = dp.tar; break;
      case AP_REG_DRW: dp.posted = mem_read_sized(dp.tar, size); tar_advance(size); break;
      default: return 4;
    }
    return ACK_OK;
  }
  switch (reg) {
    case AP_REG_CSW: dp.csw = *data; break;
    case AP_REG_TAR: dp.tar = *data; break;
    case AP_REG_DRW: mem_write_sized(dp.tar, size, *data); tar_advance(size); break;
    default: return 4;
  }
  return ACK_OK;
}

void sequence_swj(uint32_t, const uint8_t *) {}
void sequence_swd(uint8_t, const uint8_t *, uint8_t *) {}

static Shadow g_shadow;
const Shadow &shadow() { return g_shadow; }
void snoop_suspend() {}
void snoop_resume() {}
void shadow_reset() { g_shadow = Shadow{}; }
bool port_is_swd() { return true; }

}  // namespace swd

// ---------------- helpers ----------------
static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
    printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

using namespace vendor;

static uint8_t req[2048], resp[2048];

// Runs one command and returns the response. `room` defaults to the whole
// request, so a test only passes it to model a truncated packet.
static uint32_t last_ret;
static uint8_t *call(std::vector<uint8_t> r, size_t room = 0) {
  memset(resp, 0xAA, sizeof(resp));
  memcpy(req, r.data(), r.size());
  last_ret = rp2040::handle_command(req, resp, room ? room : r.size());
  return resp;
}

static void put32(std::vector<uint8_t> &v, uint32_t x) {
  for (int i = 0; i < 4; i++) v.push_back((uint8_t)(x >> (8 * i)));
}
static void put16(std::vector<uint8_t> &v, uint16_t x) {
  v.push_back((uint8_t)x); v.push_back((uint8_t)(x >> 8));
}
static uint32_t get32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static std::vector<uint8_t> attach_req(uint8_t ap, uint8_t flags,
                                       uint32_t stack_top = 0,
                                       uint32_t staging = 0, uint32_t len = 0) {
  std::vector<uint8_t> v{CMD_RP_ATTACH, ap, flags};
  put32(v, stack_top); put32(v, staging); put32(v, len);
  return v;
}

static std::vector<uint8_t> erase_req(uint32_t addr, uint32_t count) {
  std::vector<uint8_t> v{CMD_RP_FLASH_ERASE};
  put32(v, addr); put32(v, count); put32(v, 65536);
  v.push_back(0xD8); put16(v, 1000);
  return v;
}

static std::vector<uint8_t> stage_req(uint32_t off, const uint8_t *data, uint16_t len) {
  std::vector<uint8_t> v{CMD_RP_FLASH_STAGE};
  put32(v, off); put16(v, len);
  for (uint16_t i = 0; i < len; i++) v.push_back(data[i]);
  return v;
}

static std::vector<uint8_t> program_req(uint32_t addr, uint32_t off, uint32_t count) {
  std::vector<uint8_t> v{CMD_RP_FLASH_PROGRAM};
  put32(v, addr); put32(v, off); put32(v, count); put16(v, 3000);
  return v;
}

static std::vector<uint8_t> crc_req(uint32_t addr, uint32_t count) {
  std::vector<uint8_t> v{CMD_RP_FLASH_CRC};
  put32(v, addr); put32(v, count);
  return v;
}

static uint32_t crc32_of(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return crc ^ 0xFFFFFFFFu;
}

static void build_rom() {
  memset(rom, 0, sizeof(rom));
  rom[0x10] = 0x4D; rom[0x11] = 0x75; rom[0x12] = 0x01;   // 'M' 'u' version 1
  rom[0x14] = (uint8_t)ROM_TABLE; rom[0x15] = (uint8_t)(ROM_TABLE >> 8);
  static const struct { char a, b; uint16_t addr; } entries[] = {
    {'D','T',FN_TRAMPOLINE},   {'D','E',FN_TRAMPOLINE_END},
    {'I','F',FN_CONNECT_FLASH},{'E','X',FN_EXIT_XIP},
    {'R','E',FN_RANGE_ERASE},  {'R','P',FN_RANGE_PROGRAM},
    {'F','C',FN_FLUSH_CACHE},  {'C','X',FN_ENTER_CMD_XIP},
  };
  uint8_t *p = rom + ROM_TABLE;
  for (const auto &e : entries) {
    *p++ = (uint8_t)e.a; *p++ = (uint8_t)e.b;
    *p++ = (uint8_t)e.addr; *p++ = (uint8_t)(e.addr >> 8);
  }
  memset(p, 0, 4);
}

static void reset_target() {
  memset(flash, 0xFF, sizeof(flash));
  memset(ram, 0, sizeof(ram));
  build_rom();
  core = Core{};
  dp = Dp{};
  xip_enabled = true;
  rom_calls = 0;
  rp2040::reset();
}

// ---------------- tests ----------------

// Nothing works before RP_Attach: a stale jump table used against a target we
// have not re-checked is worse than refusing.
static void test_needs_attach() {
  reset_target();
  const uint8_t cmds[] = {CMD_RP_FLASH_PREP, CMD_RP_FLASH_FINISH};
  uint8_t *r = call({cmds[0]});
  CHECK(r[1] == STATUS_NOT_ATTACHED, "prep before attach: status %u", r[1]);
  r = call({cmds[1], 0});
  CHECK(r[1] == STATUS_NOT_ATTACHED, "finish before attach: status %u", r[1]);
  r = call(erase_req(0, 4096));
  CHECK(r[1] == STATUS_NOT_ATTACHED, "erase before attach: status %u", r[1]);
  r = call(crc_req(0, 256));
  CHECK(r[1] == STATUS_NOT_ATTACHED, "crc before attach: status %u", r[1]);
}

static void test_attach() {
  reset_target();
  uint8_t *r = call(attach_req(0, 0x01));
  CHECK(r[0] == CMD_RP_ATTACH, "attach echoed 0x%02x", r[0]);
  CHECK(r[1] == STATUS_OK, "attach: status %u", r[1]);
  CHECK((get32(r + 6) & 0xFFFFFF) == 0x01754D, "attach magic 0x%08x", get32(r + 6));
  CHECK(get16(r + 10) == FN_TRAMPOLINE, "trampoline 0x%04x", get16(r + 10));
  CHECK(get16(r + 12) == FN_TRAMPOLINE_END, "trampoline end 0x%04x", get16(r + 12));
  CHECK(get16(r + 22) == FN_FLUSH_CACHE, "flush_cache 0x%04x", get16(r + 22));
  CHECK(get16(r + 24) == FN_ENTER_CMD_XIP, "enter_cmd_xip 0x%04x", get16(r + 24));
  CHECK(core.halted, "attach left the core running");
  CHECK(core.maskints, "attach did not mask interrupts");
  CHECK((last_ret >> 16) == 15, "attach consumed %u request bytes", last_ret >> 16);
  CHECK((last_ret & 0xFFFF) == 26, "attach produced %u response bytes", last_ret & 0xFFFF);

  // Reset-and-halt has to end with the core halted too, having actually reset.
  reset_target();
  r = call(attach_req(0, 0x02));
  CHECK(r[1] == STATUS_OK, "reset-halt attach: status %u", r[1]);
  CHECK(core.halted, "reset-halt attach left the core running");
  CHECK(core.r[15] == 0xEA, "reset-halt attach did not reset (pc 0x%08x)", core.r[15]);
}

static void test_no_bootrom() {
  reset_target();
  memset(rom + 0x10, 0, 4);                   // something that is not an RP2040
  uint8_t *r = call(attach_req(0, 0x01));
  CHECK(r[1] == STATUS_NO_BOOTROM, "attach without a bootrom: status %u", r[1]);
  r = call({CMD_RP_FLASH_PREP});
  CHECK(r[1] == STATUS_NOT_ATTACHED, "a failed attach left a session behind");
}

// The bootrom asserts its own alignment rules and there is no way back from a
// failed assert, so they are caught before the call is made.
static void test_alignment() {
  reset_target();
  CHECK(call(attach_req(0, 0x01))[1] == STATUS_OK, "attach");

  CHECK(call(erase_req(0x1000, 4096))[1] == STATUS_OK, "aligned erase rejected");
  CHECK(call(erase_req(0x800, 4096))[1] == STATUS_BAD_REQUEST, "unaligned erase addr accepted");
  CHECK(call(erase_req(0x1000, 100))[1] == STATUS_BAD_REQUEST, "unaligned erase count accepted");
  CHECK(call(erase_req(0x1000, 0))[1] == STATUS_BAD_REQUEST, "zero-length erase accepted");

  CHECK(call(program_req(0x80, 0, 256))[1] == STATUS_BAD_REQUEST, "unaligned program addr accepted");
  CHECK(call(program_req(0x100, 0, 100))[1] == STATUS_BAD_REQUEST, "unaligned program count accepted");
}

// A length that runs past the staging buffer, or past the packet it arrived
// in, is a malformed request and not a licence to write somewhere else.
static void test_bounds() {
  reset_target();
  CHECK(call(attach_req(0, 0x01, 0, 0x20030000, 1024))[1] == STATUS_OK, "attach");

  uint8_t payload[64];
  memset(payload, 0x5A, sizeof(payload));
  uint8_t *r = call(stage_req(1024 - 32, payload, 64));
  CHECK(r[1] == STATUS_BAD_REQUEST, "stage past the buffer: status %u", r[1]);
  r = call(stage_req(960, payload, 64));
  CHECK(r[1] == STATUS_OK, "stage at the end of the buffer: status %u", r[1]);
  CHECK(get16(r + 2) == 64, "stage accepted %u bytes", get16(r + 2));

  r = call(program_req(0, 768, 512));
  CHECK(r[1] == STATUS_BAD_REQUEST, "program past the staging buffer accepted");

  // A stage whose payload is not all in the packet.
  auto truncated = stage_req(0, payload, 64);
  r = call(truncated, 20);
  CHECK(r[1] == STATUS_BAD_REQUEST, "truncated stage: status %u", r[1]);
  CHECK((last_ret >> 16) == 20, "a truncated stage claimed %u request bytes, not 20", last_ret >> 16);

  // A fixed-size command that is not all there claims the rest of the packet,
  // so a batch stops here rather than reading from past its end.
  r = call(erase_req(0, 4096), 9);
  CHECK(r[1] == STATUS_BAD_REQUEST, "truncated erase: status %u", r[1]);
  CHECK((last_ret >> 16) == 9, "a truncated erase claimed %u request bytes, not 9", last_ret >> 16);
}

// The whole sequence, and the things about it that have to be true: flash is
// out of the memory map between prep and finish, the data that lands is the
// data that was staged, and a rewrite without an erase does not silently work.
static void test_flash_sequence() {
  reset_target();
  CHECK(call(attach_req(0, 0x01))[1] == STATUS_OK, "attach");

  uint8_t image[1024];
  for (size_t i = 0; i < sizeof(image); i++) image[i] = (uint8_t)(i * 7 + 3);

  CHECK(call({CMD_RP_FLASH_PREP})[1] == STATUS_OK, "prep");
  CHECK(!xip_enabled, "prep left the flash memory mapped");

  uint8_t *r = call(crc_req(0, 256));
  CHECK(r[1] != STATUS_OK, "crc succeeded while the flash was out of the map");

  CHECK(call(erase_req(0x2000, 4096))[1] == STATUS_OK, "erase");
  CHECK(call(stage_req(0, image, sizeof(image)))[1] == STATUS_OK, "stage");
  CHECK(call(program_req(0x2000, 0, sizeof(image)))[1] == STATUS_OK, "program");
  CHECK(call({CMD_RP_FLASH_FINISH, 0})[1] == STATUS_OK, "finish");
  CHECK(xip_enabled, "finish did not put the flash back in the map");
  CHECK(core.halted, "finish without the resume flag left the core running");

  CHECK(memcmp(flash + 0x2000, image, sizeof(image)) == 0, "flash does not match the image");

  r = call(crc_req(0x2000, sizeof(image)));
  CHECK(r[1] == STATUS_OK, "crc: status %u", r[1]);
  CHECK(get32(r + 2) == crc32_of(image, sizeof(image)),
        "crc 0x%08x, expected 0x%08x", get32(r + 2), crc32_of(image, sizeof(image)));

  // Programming can only clear bits, so writing the complement over the top
  // without erasing first must not reproduce it.
  uint8_t other[1024];
  for (size_t i = 0; i < sizeof(other); i++) other[i] = (uint8_t)~image[i];
  CHECK(call({CMD_RP_FLASH_PREP})[1] == STATUS_OK, "second prep");
  CHECK(call(stage_req(0, other, sizeof(other)))[1] == STATUS_OK, "second stage");
  CHECK(call(program_req(0x2000, 0, sizeof(other)))[1] == STATUS_OK, "second program");
  CHECK(call({CMD_RP_FLASH_FINISH, 0})[1] == STATUS_OK, "second finish");
  CHECK(memcmp(flash + 0x2000, other, sizeof(other)) != 0,
        "a program without an erase rewrote the sector");
}

// One round trip per range is the whole point of RP_Flash_CRC, so the range it
// will take has a ceiling, and asking for more is refused rather than clipped.
static void test_crc_limit() {
  reset_target();
  CHECK(call(attach_req(0, 0x01))[1] == STATUS_OK, "attach");
  uint8_t *r = call(crc_req(0, rp2040::CRC_MAX_LEN + 1));
  CHECK(r[1] == STATUS_BAD_REQUEST, "an oversized crc range was accepted");
  r = call(crc_req(0, 0));
  CHECK(r[1] == STATUS_BAD_REQUEST, "a zero-length crc range was accepted");
}

// A trampoline pointed at a routine that is not there halts somewhere other
// than the breakpoint, and r0 is then not a result.
static void test_call_astray() {
  reset_target();
  CHECK(call(attach_req(0, 0x01))[1] == STATUS_OK, "attach");

  std::vector<uint8_t> v{CMD_RP_CALL, 0};
  put16(v, 0x0BAD); put16(v, 500);
  for (int i = 0; i < 4; i++) put32(v, 0);
  uint8_t *r = call(v);
  CHECK(r[1] == STATUS_CALL_FAILED, "a bad rom call reported status %u", r[1]);

  // And a real one still works afterwards.
  std::vector<uint8_t> good{CMD_RP_CALL, 0};
  put16(good, FN_FLUSH_CACHE); put16(good, 500);
  for (int i = 0; i < 4; i++) put32(good, 0);
  r = call(good);
  CHECK(r[1] == STATUS_OK, "a good rom call after a bad one: status %u", r[1]);
}

static void test_core_actions() {
  reset_target();
  CHECK(call(attach_req(0, 0x01))[1] == STATUS_OK, "attach");

  uint8_t *r = call({CMD_RP_CORE, 2, 0, 0});           // resume
  CHECK(r[1] == STATUS_OK, "resume: status %u", r[1]);
  CHECK(!core.halted, "resume left the core halted");

  r = call({CMD_RP_CORE, 1, 0xE8, 0x03});              // halt, 1000ms
  CHECK(r[1] == STATUS_OK, "halt: status %u", r[1]);
  CHECK(core.halted, "halt left the core running");
  CHECK((get32(r + 2) & (1u << 17)) != 0, "halt reported dhcsr 0x%08x", get32(r + 2));

  r = call({CMD_RP_CORE, 3, 0xE8, 0x03});              // reset and run
  CHECK(r[1] == STATUS_OK, "reset-run: status %u", r[1]);
  CHECK(!core.halted, "reset-run left the core halted");

  r = call({CMD_RP_CORE, 4, 0xE8, 0x03});              // reset and halt
  CHECK(r[1] == STATUS_OK, "reset-halt: status %u", r[1]);
  CHECK(core.halted, "reset-halt left the core running");

  r = call({CMD_RP_CORE, 9, 0, 0});
  CHECK(r[1] == STATUS_BAD_REQUEST, "an unknown core action was accepted");
}

// A connect or a disconnect invalidates what we knew about the target, the
// same way it does for the RTT session.
static void test_reset_drops_session() {
  reset_target();
  CHECK(call(attach_req(0, 0x01))[1] == STATUS_OK, "attach");
  CHECK(call({CMD_RP_FLASH_PREP})[1] == STATUS_OK, "prep");
  rp2040::reset();
  CHECK(call({CMD_RP_FLASH_PREP})[1] == STATUS_NOT_ATTACHED,
        "the session survived rp2040::reset()");
}

int main() {
  test_needs_attach();
  test_attach();
  test_no_bootrom();
  test_alignment();
  test_bounds();
  test_flash_sequence();
  test_crc_limit();
  test_call_astray();
  test_core_actions();
  test_reset_drops_session();

  printf(failures ? "\n%d FAILURES\n" : "\nall checks passed\n", failures);
  return failures ? 1 : 0;
}
