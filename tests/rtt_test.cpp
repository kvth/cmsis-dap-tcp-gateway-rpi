// Test harness: a fake SWD DP/MEM-AP over a simulated target memory, driving
// the real rtt.cpp / target_mem.cpp.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>

#include "swd_port.h"
#include "rtt.h"
#include "vendor.h"

// ---------------- simulated target ----------------
static const uint32_t RAM_BASE = 0x20000000, RAM_SIZE = 0x8000;
static uint8_t ram[RAM_SIZE];
static bool ram_mapped(uint32_t a) { return a >= RAM_BASE && a < RAM_BASE + RAM_SIZE; }

struct Dp {
  uint32_t select = 0, ctrl_stat = 0xF0000040, posted = 0;
  uint32_t csw = 0x23000052, tar = 0;   // as an OpenOCD-ish client left them
  bool byte_supported = true;
} dp;

static long transfer_count = 0;

static uint32_t mem_read_sized(uint32_t addr, unsigned size) {
  uint32_t v = 0;
  unsigned n = 1u << size;
  for (unsigned i = 0; i < n; i++) {
    uint32_t a = addr + i;
    if (!ram_mapped(a)) { dp.ctrl_stat |= swd::CTRL_STICKYERR; return 0; }
    v |= (uint32_t)ram[a - RAM_BASE] << (8 * i);
  }
  // Value sits in the byte lanes matching the address.
  return v << (8 * (addr & 3u) * (size == 0 ? 1 : 0));
}

static void mem_write_sized(uint32_t addr, unsigned size, uint32_t v) {
  unsigned n = 1u << size;
  if (size == 0) v >>= 8 * (addr & 3u);
  for (unsigned i = 0; i < n; i++) {
    uint32_t a = addr + i;
    if (!ram_mapped(a)) { dp.ctrl_stat |= swd::CTRL_STICKYERR; return; }
    ram[a - RAM_BASE] = (uint8_t)(v >> (8 * i));
  }
}

static void tar_advance(unsigned size) {
  unsigned inc = 0;
  switch ((dp.csw & swd::CSW_ADDRINC_MASK) >> 4) {
    case 1: inc = 1u << size; break;
    case 2: inc = 4; break;
    default: inc = 0; break;
  }
  if (inc) dp.tar = (dp.tar & ~1023u) | ((dp.tar + inc) & 1023u);
}

namespace swd {

uint8_t transfer(uint32_t request, uint32_t *data) {
  transfer_count++;
  const uint32_t reg = request & REQ_ADDR;
  const bool is_ap = request & REQ_APnDP;
  const bool is_read = request & REQ_RnW;

  if (!is_ap) {
    if (is_read) {
      switch (reg) {
        case DP_REG_CTRL_STAT: if (data) *data = dp.ctrl_stat; return ACK_OK;
        case DP_REG_RDBUFF:    if (data) *data = dp.posted;    return ACK_OK;
        default: return 4;  // FAULT
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

  // AP access. A sticky error blocks further AP traffic until cleared.
  if (dp.ctrl_stat & CTRL_STICKY_ANY) return 4;  // FAULT
  if (((dp.select >> 4) & 0xF) != 0) return 4;   // only bank 0 modelled

  const unsigned size = dp.csw & CSW_SIZE_MASK;
  if (is_read) {
    if (data) *data = dp.posted;               // AP reads are posted
    switch (reg) {
      case AP_REG_CSW: dp.posted = dp.csw; break;
      case AP_REG_TAR: dp.posted = dp.tar; break;
      case AP_REG_DRW: dp.posted = mem_read_sized(dp.tar, size); tar_advance(size); break;
      default: return 4;
    }
    return ACK_OK;
  }
  switch (reg) {
    case AP_REG_CSW: {
      uint32_t v = *data;
      if (!dp.byte_supported && (v & CSW_SIZE_MASK) != CSW_SIZE_WORD)
        v = (v & ~CSW_SIZE_MASK) | CSW_SIZE_WORD;   // size field is RO here
      dp.csw = v;
      break;
    }
    case AP_REG_TAR: dp.tar = *data; break;
    case AP_REG_DRW: mem_write_sized(dp.tar, size, *data); tar_advance(size); break;
    default: return 4;
  }
  return ACK_OK;
}

// dp_connect.cpp links into this test but is not exercised by it.
void sequence_swj(uint32_t, const uint8_t *) {}
void sequence_swd(uint8_t, const uint8_t *, uint8_t *) {}

Shadow g_shadow;
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

static void poke32(uint32_t a, uint32_t v) { for (int i=0;i<4;i++) ram[a-RAM_BASE+i] = (uint8_t)(v>>(8*i)); }
static uint32_t peek32(uint32_t a) { uint32_t v=0; for (int i=0;i<4;i++) v |= (uint32_t)ram[a-RAM_BASE+i]<<(8*i); return v; }

static uint8_t req[2048], resp[8192];

static uint32_t call(std::vector<uint8_t> r, size_t room = 1024) {
  memset(resp, 0xAA, sizeof(resp));
  memcpy(req, r.data(), r.size());
  return rtt::handle_command(req, resp, r.size(), room);
}

static std::vector<uint8_t> cmd_start(uint8_t ap, uint32_t addr, uint32_t len) {
  return {vendor::CMD_RTT_START, ap, (uint8_t)addr, (uint8_t)(addr>>8), (uint8_t)(addr>>16), (uint8_t)(addr>>24),
          (uint8_t)len, (uint8_t)(len>>8), (uint8_t)(len>>16), (uint8_t)(len>>24)};
}

// ---------------- control block layout ----------------
static const uint32_t CB = 0x20001000;
static const uint32_t UP0_BUF = 0x20002000, UP0_SIZE = 64;
static const uint32_t DN0_BUF = 0x20003001, DN0_SIZE = 32;   // deliberately unaligned
static const uint32_t NAME_UP = 0x20001200, NAME_DN = 0x20001210;

static uint32_t up0_desc()  { return CB + 24 + 0 * 24; }
static uint32_t dn0_desc()  { return CB + 24 + 2 * 24; }   // 2 up buffers first

static void build_cb() {
  memset(ram, 0, sizeof(ram));
  memcpy(ram + (CB - RAM_BASE), "SEGGER RTT", 11);
  poke32(CB + 16, 2);   // MaxNumUpBuffers
  poke32(CB + 20, 2);   // MaxNumDownBuffers

  memcpy(ram + (NAME_UP - RAM_BASE), "Terminal", 9);
  memcpy(ram + (NAME_DN - RAM_BASE), "Terminal", 9);

  poke32(up0_desc() +  0, NAME_UP);
  poke32(up0_desc() +  4, UP0_BUF);
  poke32(up0_desc() +  8, UP0_SIZE);
  poke32(up0_desc() + 12, 0);          // WrOff
  poke32(up0_desc() + 16, 0);          // RdOff
  poke32(up0_desc() + 20, 0);          // Flags

  poke32(dn0_desc() +  0, NAME_DN);
  poke32(dn0_desc() +  4, DN0_BUF);
  poke32(dn0_desc() +  8, DN0_SIZE);
  // up[1] and down[1] stay all zero: declared but never given storage
}

// Pretend the target appended `s` to the up buffer.
static void target_emit(const char *s) {
  uint32_t wr = peek32(up0_desc() + 12);
  for (const char *p = s; *p; p++) {
    ram[UP0_BUF - RAM_BASE + wr] = (uint8_t)*p;
    wr = (wr + 1) % UP0_SIZE;
  }
  poke32(up0_desc() + 12, wr);
}

// Pretend the target consumed everything in the down buffer, returning it.
static std::string target_drain() {
  uint32_t wr = peek32(dn0_desc() + 12), rd = peek32(dn0_desc() + 16);
  std::string out;
  while (rd != wr) { out += (char)ram[DN0_BUF - RAM_BASE + rd]; rd = (rd + 1) % DN0_SIZE; }
  poke32(dn0_desc() + 16, rd);
  return out;
}

static std::string do_read(unsigned ch, unsigned max, size_t room = 1024, uint8_t *status = nullptr) {
  uint32_t n = call({vendor::CMD_RTT_READ, (uint8_t)ch, (uint8_t)max, (uint8_t)(max>>8)}, room);
  CHECK((n >> 16) == 4, "read request len %u", n >> 16);
  if (status) *status = resp[1];
  uint16_t len = (uint16_t)(resp[2] | (resp[3] << 8));
  CHECK((n & 0xFFFF) == 4u + len, "read response len %u vs %u", n & 0xFFFF, 4u + len);
  return std::string((char *)resp + 4, len);
}

static size_t do_write(unsigned ch, const std::string &s, uint8_t *status = nullptr) {
  std::vector<uint8_t> r = {vendor::CMD_RTT_WRITE, (uint8_t)ch, (uint8_t)s.size(), (uint8_t)(s.size()>>8)};
  r.insert(r.end(), s.begin(), s.end());
  uint32_t n = call(r);
  CHECK((n >> 16) == 4 + s.size(), "write request len %u vs %zu", n >> 16, 4 + s.size());
  CHECK((n & 0xFFFF) == 4, "write response len %u", n & 0xFFFF);
  if (status) *status = resp[1];
  return (size_t)(resp[2] | (resp[3] << 8));
}

int main() {
  // --- start: exact address ---
  build_cb();
  uint32_t n = call(cmd_start(0, CB, 0));
  CHECK((n >> 16) == 10, "start request len %u", n >> 16);
  CHECK((n & 0xFFFF) == 8, "start response len %u", n & 0xFFFF);
  CHECK(resp[1] == vendor::STATUS_OK, "start status %u", resp[1]);
  uint32_t got_cb = resp[2] | (resp[3]<<8) | (resp[4]<<16) | ((uint32_t)resp[5]<<24);
  CHECK(got_cb == CB, "cb 0x%08x", got_cb);
  CHECK(resp[6] == 2 && resp[7] == 2, "counts %u/%u", resp[6], resp[7]);

  // --- start: search a range that runs off the end of RAM ---
  rtt::reset();
  dp.ctrl_stat = 0xF0000040;
  call(cmd_start(0, RAM_BASE, RAM_SIZE + 0x2000));
  CHECK(resp[1] == vendor::STATUS_OK, "search status %u", resp[1]);
  got_cb = resp[2] | (resp[3]<<8) | (resp[4]<<16) | ((uint32_t)resp[5]<<24);
  CHECK(got_cb == CB, "search cb 0x%08x", got_cb);
  CHECK((dp.ctrl_stat & swd::CTRL_STICKY_ANY) == 0, "sticky left set after search");

  // --- start: nothing there ---
  {
    uint8_t save[16]; memcpy(save, ram + (CB - RAM_BASE), 16);
    memset(ram + (CB - RAM_BASE), 0, 16);
    call(cmd_start(0, RAM_BASE, 0x1000));
    CHECK(resp[1] == vendor::STATUS_NOT_FOUND, "absent cb status %u", resp[1]);
    memcpy(ram + (CB - RAM_BASE), save, 16);
    call(cmd_start(0, CB, 0));
    CHECK(resp[1] == vendor::STATUS_OK, "restart status %u", resp[1]);
  }

  // --- status ---
  n = call({vendor::CMD_RTT_STATUS, 0, 0});
  CHECK((n >> 16) == 3, "status request len %u", n >> 16);
  CHECK(resp[1] == vendor::STATUS_OK, "status status %u", resp[1]);
  CHECK((resp[2] | (resp[3]<<8) | (resp[4]<<16) | ((uint32_t)resp[5]<<24)) == UP0_BUF, "status buffer");
  CHECK((resp[6] | (resp[7]<<8)) == UP0_SIZE, "status size");
  CHECK(resp[22] == 8, "name len %u", resp[22]);
  CHECK(memcmp(resp + 23, "Terminal", 8) == 0, "name mismatch");
  CHECK((n & 0xFFFF) == 2 + 21 + 8, "status response len %u", n & 0xFFFF);

  // --- status: unconfigured and out-of-range channels ---
  call({vendor::CMD_RTT_STATUS, 0, 1});
  CHECK(resp[1] == vendor::STATUS_OK, "up[1] status %u", resp[1]);   // readable, just empty
  call({vendor::CMD_RTT_STATUS, 0, 2});
  CHECK(resp[1] == vendor::STATUS_BAD_CHANNEL, "up[2] status %u", resp[1]);
  call({vendor::CMD_RTT_STATUS, 2, 0});
  CHECK(resp[1] == vendor::STATUS_BAD_REQUEST, "bad direction status %u", resp[1]);

  // --- read: empty, then simple ---
  CHECK(do_read(0, 512) == "", "empty read");
  target_emit("hello world");
  CHECK(do_read(0, 512) == "hello world", "simple read");
  CHECK(peek32(up0_desc() + 16) == 11, "RdOff after read = %u", peek32(up0_desc() + 16));
  CHECK(do_read(0, 512) == "", "drained read");

  // --- read: wrapping the ring ---
  {
    std::string filler(50, 'x');
    target_emit(filler.c_str());       // WrOff 11 -> 61
    CHECK(do_read(0, 512) == filler, "pre-wrap read");
    target_emit("ABCDEFGHIJ");         // 61 -> 7, wrapping
    std::string got = do_read(0, 512);
    CHECK(got == "ABCDEFGHIJ", "wrapped read got \"%s\"", got.c_str());
    CHECK(peek32(up0_desc() + 16) == 7, "RdOff after wrap = %u", peek32(up0_desc() + 16));
  }

  // --- read: max_len and response room clamp ---
  target_emit("0123456789");
  CHECK(do_read(0, 4) == "0123", "max_len clamp");
  CHECK(do_read(0, 512, 4 + 3) == "456", "response room clamp");
  CHECK(do_read(0, 512) == "789", "remainder");

  // --- read: unconfigured / bad channel ---
  {
    uint8_t st = 0;
    do_read(1, 16, 1024, &st);
    CHECK(st == vendor::STATUS_NO_BUFFER, "up[1] read status %u", st);
    do_read(9, 16, 1024, &st);
    CHECK(st == vendor::STATUS_BAD_CHANNEL, "up[9] read status %u", st);
  }

  // --- read: corrupt offsets ---
  {
    uint32_t save = peek32(up0_desc() + 12);
    poke32(up0_desc() + 12, UP0_SIZE + 5);
    uint8_t st = 0;
    do_read(0, 16, 1024, &st);
    CHECK(st == vendor::STATUS_CORRUPT, "corrupt read status %u", st);
    poke32(up0_desc() + 12, save);
  }

  // --- write: simple, into an unaligned buffer ---
  CHECK(do_write(0, "ping") == 4, "simple write");
  CHECK(peek32(dn0_desc() + 12) == 4, "WrOff after write = %u", peek32(dn0_desc() + 12));
  CHECK(target_drain() == "ping", "target saw the write");

  // --- write: fills the ring, one slot short, then wraps ---
  {
    std::string big(DN0_SIZE + 10, 'z');
    size_t took = do_write(0, big);
    CHECK(took == DN0_SIZE - 1, "full-ring write took %zu (want %u)", took, DN0_SIZE - 1);
    CHECK(target_drain() == std::string(DN0_SIZE - 1, 'z'), "target saw the full ring");
    CHECK(do_write(0, "wrapped-payload-here") == 20, "post-wrap write");
    CHECK(target_drain() == "wrapped-payload-here", "target saw the wrapped write");
  }

  // --- write: nothing fits ---
  {
    std::string big(DN0_SIZE - 1, 'q');
    CHECK(do_write(0, big) == DN0_SIZE - 1, "fill");
    CHECK(do_write(0, "more") == 0, "write into a full ring");
    target_drain();
  }

  // --- write: bad channel, not started ---
  {
    uint8_t st = 0;
    do_write(1, "x", &st);
    CHECK(st == vendor::STATUS_NO_BUFFER, "down[1] write status %u", st);
    call({vendor::CMD_RTT_STOP});
    CHECK(resp[1] == vendor::STATUS_OK, "stop status %u", resp[1]);
    do_write(0, "x", &st);
    CHECK(st == vendor::STATUS_NOT_STARTED, "stopped write status %u", st);
    do_read(0, 4, 1024, &st);
    CHECK(st == vendor::STATUS_NOT_STARTED, "stopped read status %u", st);
  }

  // --- an AP with no byte-size support still reads/writes aligned data ---
  {
    call(cmd_start(0, CB, 0));
    dp.byte_supported = false;
    target_emit("aligned-payload!");         // starts at an aligned RdOff
    uint8_t st = 0;
    std::string got = do_read(0, 512, 1024, &st);
    CHECK(st == vendor::STATUS_UNSUPPORTED || got == "aligned-payload!",
          "no-byte-AP read status %u got \"%s\"", st, got.c_str());
    dp.byte_supported = true;
  }

  // --- transfer cost: a 1 kB read should not be one transfer per byte ---
  {
    build_cb();
    call(cmd_start(0, CB, 0));
    target_emit(std::string(48, 'k').c_str());
    transfer_count = 0;
    do_read(0, 512);
    printf("transfers for a 48-byte read: %ld\n", transfer_count);
  }

  // --- malformed requests stay inside their packet ---
  {
    build_cb();
    call(cmd_start(0, CB, 0));

    // A write whose length runs past the end of the request packet.
    memset(req, 0, sizeof(req));
    req[0] = vendor::CMD_RTT_WRITE; req[1] = 0; req[2] = 0xFF; req[3] = 0xFF;
    uint32_t n = rtt::handle_command(req, resp, 16, 1024);
    CHECK(resp[1] == vendor::STATUS_BAD_REQUEST, "overlong write status %u", resp[1]);
    CHECK((n >> 16) == 16, "overlong write consumed %u of 16", n >> 16);

    // A command cut off before its fixed fields.
    req[0] = vendor::CMD_RTT_START;
    n = rtt::handle_command(req, resp, 4, 1024);
    CHECK(resp[0] == vendor::CMD_RTT_START && resp[1] == vendor::STATUS_BAD_REQUEST,
          "truncated start: id %u status %u", resp[0], resp[1]);
    CHECK((n >> 16) == 4, "truncated start consumed %u of 4", n >> 16);

    // A write that exactly fills what is left is fine.
    req[0] = vendor::CMD_RTT_WRITE; req[1] = 0; req[2] = 3; req[3] = 0;
    req[4] = 'x'; req[5] = 'y'; req[6] = 'z';
    n = rtt::handle_command(req, resp, 7, 1024);
    CHECK(resp[1] == vendor::STATUS_OK, "exact-fit write status %u", resp[1]);
    CHECK((n >> 16) == 7, "exact-fit write consumed %u of 7", n >> 16);
    CHECK(target_drain() == "xyz", "exact-fit write payload");
  }

  printf(failures ? "\n%d FAILURES\n" : "\nall checks passed\n", failures);
  return failures != 0;
}
