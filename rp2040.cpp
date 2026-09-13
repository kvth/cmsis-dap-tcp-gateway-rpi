/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RP2040 flash programming vendor commands. See rp2040.h for the wire format;
 * this file is the bootrom call mechanism it runs on.
 *
 * The sequence is OpenOCD's, from src/flash/nor/rp2040.c: find the bootrom
 * function table, then for each routine set r0..r3 and r7 and run the
 * bootrom's debug_trampoline, which is `blx r7` followed by a `bkpt`. The
 * breakpoint is how the call ends, so "the call returned" is "the core halted
 * with PC at debug_trampoline_end".
 */

#include "rp2040.h"

#include <string.h>
#include <time.h>
#include <unistd.h>

#include "logging.h"
#include "target_mem.h"
#include "vendor.h"

namespace rp2040 {

using namespace vendor;

namespace {

// --- the bootrom -----------------------------------------------------------

// 'M', 'u', then a version byte we ignore.
constexpr uint32_t BOOTROM_MAGIC      = 0x01754DU;
constexpr uint32_t BOOTROM_MAGIC_ADDR = 0x00000010U;
// The function table pointer is the u16 that follows the magic.
constexpr uint32_t BOOTROM_TABLE_PTR  = BOOTROM_MAGIC_ADDR + 4U;

// A table entry is a two-character tag and a 16-bit address, and a zero tag
// ends the table. The bootrom is 16 kB, so a table that has not ended by then
// is one we are walking in the wrong place.
constexpr unsigned MAX_TABLE_ENTRIES = 4096U;

// The entry points, in the order RP_Attach reports them.
enum JumpIndex : unsigned {
  JUMP_TRAMPOLINE = 0,
  JUMP_TRAMPOLINE_END,
  JUMP_CONNECT_INTERNAL_FLASH,
  JUMP_FLASH_EXIT_XIP,
  JUMP_FLASH_RANGE_ERASE,
  JUMP_FLASH_RANGE_PROGRAM,
  JUMP_FLASH_FLUSH_CACHE,
  JUMP_FLASH_ENTER_CMD_XIP,
  JUMP_COUNT,
};

constexpr uint16_t make_tag(char a, char b) {
  return (uint16_t)(((uint16_t)(uint8_t)b << 8) | (uint16_t)(uint8_t)a);
}

// Same tags as OpenOCD's FUNC_*, in the same order as JumpIndex.
constexpr uint16_t JUMP_TAGS[JUMP_COUNT] = {
  make_tag('D', 'T'),   // debug_trampoline
  make_tag('D', 'E'),   // debug_trampoline_end
  make_tag('I', 'F'),   // connect_internal_flash
  make_tag('E', 'X'),   // flash_exit_xip
  make_tag('R', 'E'),   // flash_range_erase
  make_tag('R', 'P'),   // flash_range_program
  make_tag('F', 'C'),   // flash_flush_cache
  make_tag('C', 'X'),   // flash_enter_cmd_xip
};

// --- flash geometry --------------------------------------------------------

constexpr uint32_t FLASH_XIP_BASE = 0x10000000U;
constexpr uint32_t FLASH_PAGE     = 256U;
constexpr uint32_t FLASH_SECTOR   = 4096U;

// --- ARMv6-M debug registers -----------------------------------------------

constexpr uint32_t REG_AIRCR = 0xE000ED0CU;
constexpr uint32_t REG_DHCSR = 0xE000EDF0U;
constexpr uint32_t REG_DCRSR = 0xE000EDF4U;
constexpr uint32_t REG_DCRDR = 0xE000EDF8U;
constexpr uint32_t REG_DEMCR = 0xE000EDFCU;

constexpr uint32_t DHCSR_DBGKEY     = 0xA05FU << 16;
constexpr uint32_t DHCSR_C_DEBUGEN  = 1U << 0;
constexpr uint32_t DHCSR_C_HALT     = 1U << 1;
constexpr uint32_t DHCSR_C_MASKINTS = 1U << 3;
constexpr uint32_t DHCSR_S_REGRDY   = 1U << 16;
constexpr uint32_t DHCSR_S_HALT     = 1U << 17;
constexpr uint32_t DHCSR_S_LOCKUP   = 1U << 19;
constexpr uint32_t DHCSR_S_RESET_ST = 1U << 25;

constexpr uint32_t DCRSR_WRITE = 1U << 16;
constexpr uint32_t DCRSR_MASK  = 0x1FU;

constexpr uint32_t DEMCR_VC_CORERESET = 1U << 0;

constexpr uint32_t AIRCR_VECTKEY     = 0x05FAU << 16;
constexpr uint32_t AIRCR_SYSRESETREQ = 1U << 2;

constexpr uint32_t XPSR_THUMB = 1U << 24;

// DCRSR register selectors we use.
constexpr uint32_t CORE_R0   = 0U;
constexpr uint32_t CORE_R7   = 7U;
constexpr uint32_t CORE_SP   = 13U;
constexpr uint32_t CORE_LR   = 14U;
constexpr uint32_t CORE_PC   = 15U;
constexpr uint32_t CORE_XPSR = 16U;
constexpr uint32_t CORE_MSP  = 17U;

// --- session ---------------------------------------------------------------

// Defaults for where the bootrom's stack and our staging buffer live. The
// stack is the top of SRAM5, the 4 kB bank the RP2040 keeps at the end of the
// map; the staging buffer sits in the main 256 kB striped region, clear of it.
constexpr uint32_t DEFAULT_STACK_TOP    = 0x20042000U;
constexpr uint32_t DEFAULT_STAGING_ADDR = 0x20020000U;
constexpr uint32_t DEFAULT_STAGING_LEN  = 64U * 1024U;

struct Session {
  bool     attached;
  uint8_t  ap;
  uint32_t stack_top;
  uint32_t staging_addr;
  uint32_t staging_len;
  uint16_t jump[JUMP_COUNT];
};

Session session;

// --- timing ----------------------------------------------------------------

// Fine enough that a ROM call that finishes quickly is noticed quickly, coarse
// enough that the poll itself is not most of the SWD traffic.
constexpr useconds_t POLL_INTERVAL_US = 200U;

// A core register access settles in a handful of core clocks. This is only
// here so a target that has stopped answering does not wedge the gateway.
constexpr unsigned REGRDY_TIMEOUT_MS = 100U;

// What OpenOCD gives the bracketing ROM calls, which touch the QSPI interface
// but do not wait on the flash part itself.
constexpr unsigned ROM_CALL_TIMEOUT_MS = 1000U;

uint64_t now_ms() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ((uint64_t)ts.tv_sec * 1000U) + (uint64_t)(ts.tv_nsec / 1000000);
}

unsigned clamp_timeout(unsigned ms) {
  if (ms == 0U) {
    return ROM_CALL_TIMEOUT_MS;
  }
  return (ms > MAX_TIMEOUT_MS) ? MAX_TIMEOUT_MS : ms;
}

// --- little endian helpers --------------------------------------------------

uint16_t load_le16(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint32_t load_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint8_t *store_le16(uint8_t *p, uint16_t v) {
  *p++ = (uint8_t)v;
  *p++ = (uint8_t)(v >> 8);
  return p;
}

uint8_t *store_le32(uint8_t *p, uint32_t v) {
  *p++ = (uint8_t)v;
  *p++ = (uint8_t)(v >> 8);
  *p++ = (uint8_t)(v >> 16);
  *p++ = (uint8_t)(v >> 24);
  return p;
}

Status from_mem(target_mem::Status s) {
  switch (s) {
    case target_mem::Status::ok:             return STATUS_OK;
    case target_mem::Status::not_connected:  return STATUS_NOT_CONNECTED;
    case target_mem::Status::sticky_error:   return STATUS_DAP_BUSY;
    case target_mem::Status::transfer_fault: return STATUS_TRANSFER;
    case target_mem::Status::unsupported:    return STATUS_UNSUPPORTED;
  }
  return STATUS_TRANSFER;
}

uint32_t reply(uint8_t *response, uint8_t cmd, Status status, size_t body_len,
               uint32_t request_len) {
  response[0] = cmd;
  response[1] = (uint8_t)status;
  if (status != STATUS_OK) {
    memset(response + 2, 0, body_len);
  }
  return (request_len << 16) | (uint32_t)(2U + body_len);
}

// --- core control ----------------------------------------------------------

Status read_dhcsr(target_mem::Burst &bus, uint32_t *out) {
  return from_mem(bus.read_u32(REG_DHCSR, out));
}

Status write_dhcsr(target_mem::Burst &bus, uint32_t bits) {
  return from_mem(bus.write_u32(REG_DHCSR, DHCSR_DBGKEY | bits));
}

// Wait for the core register transfer started by the last DCRSR write.
Status wait_regrdy(target_mem::Burst &bus) {
  const uint64_t deadline = now_ms() + REGRDY_TIMEOUT_MS;
  for (;;) {
    uint32_t dhcsr = 0U;
    const Status st = read_dhcsr(bus, &dhcsr);
    if (st != STATUS_OK) {
      return st;
    }
    if ((dhcsr & DHCSR_S_REGRDY) != 0U) {
      return STATUS_OK;
    }
    if (now_ms() >= deadline) {
      return STATUS_TIMEOUT;
    }
    usleep(POLL_INTERVAL_US);
  }
}

Status core_reg_write(target_mem::Burst &bus, uint32_t reg, uint32_t value) {
  Status st = from_mem(bus.write_u32(REG_DCRDR, value));
  if (st != STATUS_OK) {
    return st;
  }
  st = from_mem(bus.write_u32(REG_DCRSR, DCRSR_WRITE | (reg & DCRSR_MASK)));
  if (st != STATUS_OK) {
    return st;
  }
  return wait_regrdy(bus);
}

Status core_reg_read(target_mem::Burst &bus, uint32_t reg, uint32_t *out) {
  Status st = from_mem(bus.write_u32(REG_DCRSR, reg & DCRSR_MASK));
  if (st != STATUS_OK) {
    return st;
  }
  st = wait_regrdy(bus);
  if (st != STATUS_OK) {
    return st;
  }
  return from_mem(bus.read_u32(REG_DCRDR, out));
}

Status wait_halted(target_mem::Burst &bus, unsigned timeout_ms,
                   uint32_t *dhcsr_out) {
  const uint64_t deadline = now_ms() + timeout_ms;
  for (;;) {
    uint32_t dhcsr = 0U;
    const Status st = read_dhcsr(bus, &dhcsr);
    if (st != STATUS_OK) {
      return st;
    }
    if (dhcsr_out != nullptr) {
      *dhcsr_out = dhcsr;
    }
    if ((dhcsr & DHCSR_S_HALT) != 0U) {
      return STATUS_OK;
    }
    if (now_ms() >= deadline) {
      return STATUS_TIMEOUT;
    }
    usleep(POLL_INTERVAL_US);
  }
}

Status core_halt(target_mem::Burst &bus, unsigned timeout_ms,
                 uint32_t *dhcsr_out) {
  Status st = write_dhcsr(bus, DHCSR_C_DEBUGEN | DHCSR_C_HALT);
  if (st != STATUS_OK) {
    return st;
  }
  st = wait_halted(bus, timeout_ms, dhcsr_out);
  if (st != STATUS_OK) {
    return st;
  }
  // C_MASKINTS may only be changed while the core is halted, so it goes in on
  // a second write. Interrupts stay masked for the rest of the session: a
  // bootrom routine running off a stack we picked has no business being
  // interrupted by the application's handlers.
  return write_dhcsr(bus, DHCSR_C_DEBUGEN | DHCSR_C_HALT | DHCSR_C_MASKINTS);
}

Status core_resume(target_mem::Burst &bus, bool mask_ints) {
  const uint32_t bits =
      DHCSR_C_DEBUGEN | (mask_ints ? DHCSR_C_MASKINTS : 0U);
  return write_dhcsr(bus, bits);
}

// Reset through AIRCR.SYSRESETREQ, optionally catching the core at the reset
// vector with DEMCR.VC_CORERESET.
//
// Transfers fail while the target is in reset, so this tolerates faults for as
// long as the budget lasts rather than reporting the first one. It also
// insists on seeing S_RESET_ST -- a sticky bit that clears when read -- before
// calling it done, because otherwise "the core is halted" would be satisfied
// by the core we halted ourselves on the way in.
Status core_reset(target_mem::Burst &bus, bool halt_after, unsigned timeout_ms,
                  uint32_t *dhcsr_out) {
  if (halt_after) {
    // Best effort: a core that is already wedged still gets reset below.
    (void)write_dhcsr(bus, DHCSR_C_DEBUGEN | DHCSR_C_HALT);
  } else {
    (void)write_dhcsr(bus, DHCSR_C_DEBUGEN);
  }

  uint32_t demcr = 0U;
  Status st = from_mem(bus.read_u32(REG_DEMCR, &demcr));
  if (st != STATUS_OK) {
    return st;
  }
  const uint32_t armed = halt_after ? (demcr | DEMCR_VC_CORERESET)
                                    : (demcr & ~DEMCR_VC_CORERESET);
  st = from_mem(bus.write_u32(REG_DEMCR, armed));
  if (st != STATUS_OK) {
    return st;
  }

  // The reset can land before this write is acknowledged, so a fault here says
  // nothing about whether it worked.
  if (bus.write_u32(REG_AIRCR, AIRCR_VECTKEY | AIRCR_SYSRESETREQ) !=
      target_mem::Status::ok) {
    (void)bus.clear_sticky();
  }

  const uint64_t deadline = now_ms() + timeout_ms;
  bool     saw_reset = false;
  bool     done      = false;
  uint32_t dhcsr     = 0U;

  while (!done) {
    if (bus.read_u32(REG_DHCSR, &dhcsr) != target_mem::Status::ok) {
      (void)bus.clear_sticky();
    } else if ((dhcsr & DHCSR_S_RESET_ST) != 0U) {
      // Either still in reset or just out of it; the read has cleared the bit,
      // so the next one tells us which.
      saw_reset = true;
    } else if (saw_reset) {
      done = !halt_after || ((dhcsr & DHCSR_S_HALT) != 0U);
    }

    if (!done) {
      if (now_ms() >= deadline) {
        break;
      }
      usleep(POLL_INTERVAL_US);
    }
  }

  // Disarm the vector catch whatever happened, or every later reset would stop
  // at the reset vector too.
  (void)bus.write_u32(REG_DEMCR, demcr & ~DEMCR_VC_CORERESET);

  if (dhcsr_out != nullptr) {
    *dhcsr_out = dhcsr;
  }
  if (!done) {
    return saw_reset ? STATUS_TIMEOUT : STATUS_TRANSFER;
  }
  if (halt_after) {
    // Same two-step as core_halt(): mask interrupts now that it is stopped.
    return write_dhcsr(bus,
                       DHCSR_C_DEBUGEN | DHCSR_C_HALT | DHCSR_C_MASKINTS);
  }
  return write_dhcsr(bus, DHCSR_C_DEBUGEN);
}

// --- bootrom lookup --------------------------------------------------------

Status read_u16(target_mem::Burst &bus, uint32_t addr, uint16_t *out) {
  uint8_t raw[2];
  const target_mem::Status st = bus.read(addr, raw, sizeof(raw));
  if (st != target_mem::Status::ok) {
    return from_mem(st);
  }
  *out = load_le16(raw);
  return STATUS_OK;
}

Status lookup_symbol(target_mem::Burst &bus, uint16_t table, uint16_t tag,
                     uint16_t *out) {
  for (unsigned i = 0U; i < MAX_TABLE_ENTRIES; ++i) {
    const uint32_t entry = (uint32_t)table + ((uint32_t)i * 4U);
    uint16_t entry_tag = 0U;
    const Status st = read_u16(bus, entry, &entry_tag);
    if (st != STATUS_OK) {
      return st;
    }
    if (entry_tag == 0U) {
      return STATUS_NOT_FOUND;
    }
    if (entry_tag == tag) {
      return read_u16(bus, entry + 2U, out);
    }
  }
  return STATUS_NOT_FOUND;
}

Status load_jump_table(target_mem::Burst &bus, uint32_t *magic_out,
                       uint16_t jump[JUMP_COUNT]) {
  uint32_t magic = 0U;
  Status st = from_mem(bus.read_u32(BOOTROM_MAGIC_ADDR, &magic));
  if (st != STATUS_OK) {
    return st;
  }
  *magic_out = magic;
  if ((magic & 0x00FFFFFFU) != BOOTROM_MAGIC) {
    return STATUS_NO_BOOTROM;
  }

  uint16_t table = 0U;
  st = read_u16(bus, BOOTROM_TABLE_PTR, &table);
  if (st != STATUS_OK) {
    return st;
  }

  for (unsigned i = 0U; i < JUMP_COUNT; ++i) {
    st = lookup_symbol(bus, table, JUMP_TAGS[i], &jump[i]);
    if (st != STATUS_OK) {
      return st;
    }
  }

  // The trampoline addresses are branch targets for us, not for a `bx`, so the
  // Thumb bit comes off -- the same masking OpenOCD does. The routine
  // addresses keep theirs: they are reached through `blx r7`.
  jump[JUMP_TRAMPOLINE]     &= (uint16_t)~1U;
  jump[JUMP_TRAMPOLINE_END] &= (uint16_t)~1U;
  return STATUS_OK;
}

// --- calling a bootrom routine ---------------------------------------------

// Run `func` through the debug trampoline. The core must already be halted.
Status rom_call(target_mem::Burst &bus, uint16_t func, const uint32_t *args,
                unsigned n_args, unsigned timeout_ms, uint32_t *r0_out) {
  const uint32_t tramp     = session.jump[JUMP_TRAMPOLINE];
  const uint32_t tramp_end = session.jump[JUMP_TRAMPOLINE_END];

  uint32_t dhcsr = 0U;
  Status st = read_dhcsr(bus, &dhcsr);
  if (st != STATUS_OK) {
    return st;
  }
  if ((dhcsr & DHCSR_S_HALT) == 0U) {
    return STATUS_NOT_HALTED;
  }

  for (unsigned i = 0U; i < n_args; ++i) {
    st = core_reg_write(bus, CORE_R0 + i, args[i]);
    if (st != STATUS_OK) {
      return st;
    }
  }
  // The trampoline takes the routine in r7 and does `blx r7`.
  st = core_reg_write(bus, CORE_R7, func);
  if (st != STATUS_OK) {
    return st;
  }
  // Both stack views get the same value, so it does not matter which one the
  // core is currently using.
  if ((st = core_reg_write(bus, CORE_SP, session.stack_top)) != STATUS_OK ||
      (st = core_reg_write(bus, CORE_MSP, session.stack_top)) != STATUS_OK) {
    return st;
  }
  // Only the Thumb bit matters in xPSR here: anything left over from the
  // application (an active exception number, say) would make the return from
  // the routine mean something else.
  if ((st = core_reg_write(bus, CORE_XPSR, XPSR_THUMB)) != STATUS_OK) {
    return st;
  }
  // `blx` overwrites LR before the routine sees it; this is only a backstop
  // for a routine entered some other way.
  if ((st = core_reg_write(bus, CORE_LR, tramp_end | 1U)) != STATUS_OK) {
    return st;
  }
  if ((st = core_reg_write(bus, CORE_PC, tramp)) != STATUS_OK) {
    return st;
  }

  st = core_resume(bus, true);
  if (st != STATUS_OK) {
    return st;
  }

  st = wait_halted(bus, timeout_ms, &dhcsr);
  if (st != STATUS_OK) {
    // Put the core back under control before giving up, or the next command
    // would find it running bootrom code on a stack we chose.
    (void)write_dhcsr(bus, DHCSR_C_DEBUGEN | DHCSR_C_HALT | DHCSR_C_MASKINTS);
    LOGW_KV("rp2040 rom call timed out",
            "func=0x%04x timeout_ms=%u dhcsr=0x%08x",
            (unsigned)func, timeout_ms, (unsigned)dhcsr);
    return st;
  }

  // Re-assert C_HALT: the breakpoint stopped the core, but DHCSR still says
  // "running" until we say otherwise, and a later resume would not take.
  st = write_dhcsr(bus, DHCSR_C_DEBUGEN | DHCSR_C_HALT | DHCSR_C_MASKINTS);
  if (st != STATUS_OK) {
    return st;
  }

  // It has to have stopped on the trampoline's own breakpoint. Anywhere else
  // -- a fault, a lockup, a stray breakpoint in the application -- means the
  // routine did not run to completion and r0 is not a result.
  uint32_t pc = 0U;
  st = core_reg_read(bus, CORE_PC, &pc);
  if (st != STATUS_OK) {
    return st;
  }
  if (((pc & ~1U) != tramp_end) || ((dhcsr & DHCSR_S_LOCKUP) != 0U)) {
    LOGW_KV("rp2040 rom call went astray",
            "func=0x%04x pc=0x%08x expected=0x%08x dhcsr=0x%08x",
            (unsigned)func, (unsigned)pc, (unsigned)tramp_end,
            (unsigned)dhcsr);
    return STATUS_CALL_FAILED;
  }

  uint32_t r0 = 0U;
  st = core_reg_read(bus, CORE_R0, &r0);
  if (st != STATUS_OK) {
    return st;
  }
  if (r0_out != nullptr) {
    *r0_out = r0;
  }
  return STATUS_OK;
}

Status rom_call_indexed(target_mem::Burst &bus, JumpIndex which,
                        const uint32_t *args, unsigned n_args,
                        unsigned timeout_ms) {
  return rom_call(bus, session.jump[which], args, n_args, timeout_ms, nullptr);
}

// --- CRC-32 ----------------------------------------------------------------

// Bitwise rather than table driven: the flash read in front of it costs orders
// of magnitude more than the arithmetic, and a 1 kB table would not earn its
// place in a probe binary.
uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  for (size_t i = 0U; i < len; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0U; bit < 8U; ++bit) {
      const uint32_t mask = (uint32_t)0U - (crc & 1U);
      crc = (crc >> 1) ^ (0xEDB88320U & mask);
    }
  }
  return crc;
}

// --- commands --------------------------------------------------------------

uint32_t cmd_attach(const uint8_t *request, uint8_t *response) {
  constexpr size_t REQUEST_LEN = 1U + 1U + 1U + 4U + 4U + 4U;
  constexpr size_t BODY_LEN    = 4U + 4U + (2U * (size_t)JUMP_COUNT);

  const uint8_t  ap           = request[1];
  const uint8_t  flags        = request[2];
  const uint32_t stack_top    = load_le32(request + 3);
  const uint32_t staging_addr = load_le32(request + 7);
  const uint32_t staging_len  = load_le32(request + 11);

  const bool reset_halt = (flags & 0x02U) != 0U;
  const bool halt_first = reset_halt || ((flags & 0x01U) != 0U);

  // A stale jump table is worse than none: it would be used by the very next
  // command against a target we have not re-checked.
  session.attached = false;

  target_mem::Burst bus(ap);
  Status status = from_mem(bus.status());

  uint32_t dhcsr = 0U;
  uint32_t magic = 0U;
  uint16_t jump[JUMP_COUNT];
  memset(jump, 0, sizeof(jump));

  if (status == STATUS_OK) {
    if (reset_halt) {
      status = core_reset(bus, true, ROM_CALL_TIMEOUT_MS, &dhcsr);
    } else if (halt_first) {
      status = core_halt(bus, ROM_CALL_TIMEOUT_MS, &dhcsr);
    } else {
      status = read_dhcsr(bus, &dhcsr);
    }
  }
  if ((status == STATUS_OK) && ((dhcsr & DHCSR_S_HALT) == 0U)) {
    // Reading the bootrom does not need a halted core, but every command that
    // follows does, so failing here beats failing later with a jump table in
    // hand.
    status = STATUS_NOT_HALTED;
  }
  if (status == STATUS_OK) {
    status = load_jump_table(bus, &magic, jump);
  }

  if (status == STATUS_OK) {
    session.attached     = true;
    session.ap           = ap;
    session.stack_top    = (stack_top != 0U) ? stack_top : DEFAULT_STACK_TOP;
    session.staging_addr =
        (staging_addr != 0U) ? staging_addr : DEFAULT_STAGING_ADDR;
    session.staging_len =
        (staging_len != 0U) ? staging_len : DEFAULT_STAGING_LEN;
    memcpy(session.jump, jump, sizeof(session.jump));

    LOGI_KV("rp2040 attached",
            "ap=%u dhcsr=0x%08x rom=0x%06x trampoline=0x%04x stack=0x%08x "
            "staging=0x%08x+%u",
            (unsigned)ap, (unsigned)dhcsr, (unsigned)(magic & 0x00FFFFFFU),
            (unsigned)jump[JUMP_TRAMPOLINE], (unsigned)session.stack_top,
            (unsigned)session.staging_addr, (unsigned)session.staging_len);
  } else {
    LOGW_KV("rp2040 attach failed", "ap=%u status=%u dhcsr=0x%08x",
            (unsigned)ap, (unsigned)status, (unsigned)dhcsr);
    return reply(response, CMD_RP_ATTACH, status, BODY_LEN,
                 (uint32_t)REQUEST_LEN);
  }

  response[0] = CMD_RP_ATTACH;
  response[1] = (uint8_t)status;
  uint8_t *body = response + 2;
  body = store_le32(body, dhcsr);
  body = store_le32(body, magic);
  for (unsigned i = 0U; i < JUMP_COUNT; ++i) {
    body = store_le16(body, jump[i]);
  }
  return ((uint32_t)REQUEST_LEN << 16) | (uint32_t)(2U + BODY_LEN);
}

uint32_t cmd_core(const uint8_t *request, uint8_t *response) {
  constexpr size_t REQUEST_LEN = 1U + 1U + 2U;
  constexpr size_t BODY_LEN    = 4U;

  const uint8_t  action     = request[1];
  const unsigned timeout_ms = clamp_timeout(load_le16(request + 2));

  if (!session.attached) {
    return reply(response, CMD_RP_CORE, STATUS_NOT_ATTACHED, BODY_LEN,
                 (uint32_t)REQUEST_LEN);
  }
  if (action > 4U) {
    return reply(response, CMD_RP_CORE, STATUS_BAD_REQUEST, BODY_LEN,
                 (uint32_t)REQUEST_LEN);
  }

  target_mem::Burst bus(session.ap);
  Status   status = from_mem(bus.status());
  uint32_t dhcsr  = 0U;

  if (status == STATUS_OK) {
    switch (action) {
      case 0U: status = read_dhcsr(bus, &dhcsr); break;
      case 1U: status = core_halt(bus, timeout_ms, &dhcsr); break;
      case 2U: status = core_resume(bus, false); break;
      case 3U: status = core_reset(bus, false, timeout_ms, &dhcsr); break;
      default: status = core_reset(bus, true, timeout_ms, &dhcsr); break;
    }
  }
  // Whatever the action was, report the state it left behind rather than a
  // snapshot from the middle of it.
  if (status == STATUS_OK) {
    (void)read_dhcsr(bus, &dhcsr);
  }

  // A reset invalidates nothing we cached -- the bootrom does not move -- but
  // a core that is running again cannot take ROM calls.
  if ((status == STATUS_OK) && (action == 2U || action == 3U)) {
    LOGI_KV("rp2040 core running", "action=%u dhcsr=0x%08x", (unsigned)action,
            (unsigned)dhcsr);
  }

  if (status != STATUS_OK) {
    return reply(response, CMD_RP_CORE, status, BODY_LEN,
                 (uint32_t)REQUEST_LEN);
  }
  response[0] = CMD_RP_CORE;
  response[1] = (uint8_t)status;
  store_le32(response + 2, dhcsr);
  return ((uint32_t)REQUEST_LEN << 16) | (uint32_t)(2U + BODY_LEN);
}

uint32_t cmd_call(const uint8_t *request, uint8_t *response) {
  constexpr size_t REQUEST_LEN = 1U + 1U + 2U + 2U + 16U;
  constexpr size_t BODY_LEN    = 4U;

  const uint8_t  n_args     = request[1];
  const uint16_t func       = load_le16(request + 2);
  const unsigned timeout_ms = clamp_timeout(load_le16(request + 4));

  uint32_t args[4];
  for (unsigned i = 0U; i < 4U; ++i) {
    args[i] = load_le32(request + 6U + (i * 4U));
  }

  if (!session.attached) {
    return reply(response, CMD_RP_CALL, STATUS_NOT_ATTACHED, BODY_LEN,
                 (uint32_t)REQUEST_LEN);
  }
  if (n_args > 4U) {
    return reply(response, CMD_RP_CALL, STATUS_BAD_REQUEST, BODY_LEN,
                 (uint32_t)REQUEST_LEN);
  }

  target_mem::Burst bus(session.ap);
  Status   status = from_mem(bus.status());
  uint32_t r0     = 0U;

  if (status == STATUS_OK) {
    status = rom_call(bus, func, args, n_args, timeout_ms, &r0);
  }

  if (status != STATUS_OK) {
    return reply(response, CMD_RP_CALL, status, BODY_LEN,
                 (uint32_t)REQUEST_LEN);
  }
  response[0] = CMD_RP_CALL;
  response[1] = (uint8_t)status;
  store_le32(response + 2, r0);
  return ((uint32_t)REQUEST_LEN << 16) | (uint32_t)(2U + BODY_LEN);
}

uint32_t cmd_flash_prep(uint8_t *response) {
  constexpr size_t REQUEST_LEN = 1U;

  if (!session.attached) {
    return reply(response, CMD_RP_FLASH_PREP, STATUS_NOT_ATTACHED, 0U,
                 (uint32_t)REQUEST_LEN);
  }

  target_mem::Burst bus(session.ap);
  Status status = from_mem(bus.status());

  if (status == STATUS_OK) {
    status = rom_call_indexed(bus, JUMP_CONNECT_INTERNAL_FLASH, nullptr, 0U,
                              ROM_CALL_TIMEOUT_MS);
  }
  if (status == STATUS_OK) {
    status = rom_call_indexed(bus, JUMP_FLASH_EXIT_XIP, nullptr, 0U,
                              ROM_CALL_TIMEOUT_MS);
  }
  if (status != STATUS_OK) {
    LOGW_KV("rp2040 flash prep failed", "status=%u", (unsigned)status);
  }
  return reply(response, CMD_RP_FLASH_PREP, status, 0U, (uint32_t)REQUEST_LEN);
}

uint32_t cmd_flash_erase(const uint8_t *request, uint8_t *response) {
  constexpr size_t REQUEST_LEN = 1U + 4U + 4U + 4U + 1U + 2U;

  const uint32_t addr       = load_le32(request + 1);
  const uint32_t count      = load_le32(request + 5);
  const uint32_t block_size = load_le32(request + 9);
  const uint32_t block_cmd  = request[13];
  const unsigned timeout_ms = clamp_timeout(load_le16(request + 14));

  if (!session.attached) {
    return reply(response, CMD_RP_FLASH_ERASE, STATUS_NOT_ATTACHED, 0U,
                 (uint32_t)REQUEST_LEN);
  }
  // The bootrom asserts these itself and there is no way back from a failed
  // assert, so they are checked here where the failure is still reportable.
  if (((addr % FLASH_SECTOR) != 0U) || ((count % FLASH_SECTOR) != 0U) ||
      (count == 0U)) {
    return reply(response, CMD_RP_FLASH_ERASE, STATUS_BAD_REQUEST, 0U,
                 (uint32_t)REQUEST_LEN);
  }

  target_mem::Burst bus(session.ap);
  Status status = from_mem(bus.status());

  if (status == STATUS_OK) {
    const uint32_t args[4] = {addr, count, block_size, block_cmd};
    status = rom_call_indexed(bus, JUMP_FLASH_RANGE_ERASE, args, 4U,
                              timeout_ms);
  }
  if (status == STATUS_OK) {
    LOGD_KV("rp2040 erased", "addr=0x%08x count=%u", (unsigned)addr,
            (unsigned)count);
  } else {
    LOGW_KV("rp2040 erase failed", "addr=0x%08x count=%u status=%u",
            (unsigned)addr, (unsigned)count, (unsigned)status);
  }
  return reply(response, CMD_RP_FLASH_ERASE, status, 0U, (uint32_t)REQUEST_LEN);
}

uint32_t cmd_flash_stage(const uint8_t *request, uint8_t *response,
                         size_t request_room) {
  constexpr size_t HEADER_LEN = 1U + 4U + 2U;
  constexpr size_t BODY_LEN   = 2U;

  const uint32_t offset = load_le32(request + 1);
  const size_t   len    = load_le16(request + 5);
  const uint8_t *data   = request + HEADER_LEN;
  const uint32_t request_len = (uint32_t)(HEADER_LEN + len);

  // As with RTT_Write: a length that runs past the packet is a malformed
  // request, not licence to read past the receive buffer.
  if ((HEADER_LEN + len) > request_room) {
    return reply(response, CMD_RP_FLASH_STAGE, STATUS_BAD_REQUEST, BODY_LEN,
                 (uint32_t)request_room);
  }
  if (!session.attached) {
    return reply(response, CMD_RP_FLASH_STAGE, STATUS_NOT_ATTACHED, BODY_LEN,
                 request_len);
  }
  if (((uint64_t)offset + (uint64_t)len) > (uint64_t)session.staging_len) {
    return reply(response, CMD_RP_FLASH_STAGE, STATUS_BAD_REQUEST, BODY_LEN,
                 request_len);
  }

  target_mem::Burst bus(session.ap);
  Status status  = from_mem(bus.status());
  size_t written = 0U;

  if ((status == STATUS_OK) && (len != 0U)) {
    status = from_mem(bus.write(session.staging_addr + offset, data, len));
    if (status == STATUS_OK) {
      written = len;
    }
  }

  store_le16(response + 2, (uint16_t)written);
  response[0] = CMD_RP_FLASH_STAGE;
  response[1] = (uint8_t)status;
  return (request_len << 16) | (uint32_t)(2U + BODY_LEN);
}

uint32_t cmd_flash_program(const uint8_t *request, uint8_t *response) {
  constexpr size_t REQUEST_LEN = 1U + 4U + 4U + 4U + 2U;

  const uint32_t addr       = load_le32(request + 1);
  const uint32_t stage_off  = load_le32(request + 5);
  const uint32_t count      = load_le32(request + 9);
  const unsigned timeout_ms = clamp_timeout(load_le16(request + 13));

  if (!session.attached) {
    return reply(response, CMD_RP_FLASH_PROGRAM, STATUS_NOT_ATTACHED, 0U,
                 (uint32_t)REQUEST_LEN);
  }
  if (((addr % FLASH_PAGE) != 0U) || ((count % FLASH_PAGE) != 0U) ||
      (count == 0U) ||
      (((uint64_t)stage_off + (uint64_t)count) >
       (uint64_t)session.staging_len)) {
    return reply(response, CMD_RP_FLASH_PROGRAM, STATUS_BAD_REQUEST, 0U,
                 (uint32_t)REQUEST_LEN);
  }

  target_mem::Burst bus(session.ap);
  Status status = from_mem(bus.status());

  if (status == STATUS_OK) {
    const uint32_t args[3] = {addr, session.staging_addr + stage_off, count};
    status = rom_call_indexed(bus, JUMP_FLASH_RANGE_PROGRAM, args, 3U,
                              timeout_ms);
  }
  if (status == STATUS_OK) {
    LOGD_KV("rp2040 programmed", "addr=0x%08x count=%u", (unsigned)addr,
            (unsigned)count);
  } else {
    LOGW_KV("rp2040 program failed", "addr=0x%08x count=%u status=%u",
            (unsigned)addr, (unsigned)count, (unsigned)status);
  }
  return reply(response, CMD_RP_FLASH_PROGRAM, status, 0U,
               (uint32_t)REQUEST_LEN);
}

uint32_t cmd_flash_finish(const uint8_t *request, uint8_t *response) {
  constexpr size_t REQUEST_LEN = 1U + 1U;

  const bool resume = (request[1] & 0x01U) != 0U;

  if (!session.attached) {
    return reply(response, CMD_RP_FLASH_FINISH, STATUS_NOT_ATTACHED, 0U,
                 (uint32_t)REQUEST_LEN);
  }

  target_mem::Burst bus(session.ap);
  Status status = from_mem(bus.status());

  if (status == STATUS_OK) {
    // Both calls are made even if the flush fails: enter_cmd_xip is what puts
    // the flash back in the memory map, and a target that boots from stale
    // cache contents is still better than one that does not boot at all.
    const Status flushed = rom_call_indexed(bus, JUMP_FLASH_FLUSH_CACHE,
                                            nullptr, 0U, ROM_CALL_TIMEOUT_MS);
    const Status xip = rom_call_indexed(bus, JUMP_FLASH_ENTER_CMD_XIP, nullptr,
                                        0U, ROM_CALL_TIMEOUT_MS);
    status = (flushed != STATUS_OK) ? flushed : xip;
  }

  if ((status == STATUS_OK) && resume) {
    status = core_resume(bus, false);
  }
  if (status != STATUS_OK) {
    LOGW_KV("rp2040 flash finish failed", "status=%u", (unsigned)status);
  }
  return reply(response, CMD_RP_FLASH_FINISH, status, 0U,
               (uint32_t)REQUEST_LEN);
}

uint32_t cmd_flash_crc(const uint8_t *request, uint8_t *response) {
  constexpr size_t REQUEST_LEN = 1U + 4U + 4U;
  constexpr size_t BODY_LEN    = 4U;
  constexpr size_t CHUNK       = 512U;

  const uint32_t addr  = load_le32(request + 1);
  const uint32_t count = load_le32(request + 5);

  if (!session.attached) {
    return reply(response, CMD_RP_FLASH_CRC, STATUS_NOT_ATTACHED, BODY_LEN,
                 (uint32_t)REQUEST_LEN);
  }
  if ((count == 0U) || (count > CRC_MAX_LEN)) {
    return reply(response, CMD_RP_FLASH_CRC, STATUS_BAD_REQUEST, BODY_LEN,
                 (uint32_t)REQUEST_LEN);
  }

  target_mem::Burst bus(session.ap);
  Status   status = from_mem(bus.status());
  uint32_t crc    = 0xFFFFFFFFU;

  uint8_t chunk[CHUNK];
  for (uint32_t done = 0U; (status == STATUS_OK) && (done < count);) {
    size_t take = count - done;
    if (take > CHUNK) {
      take = CHUNK;
    }
    status = from_mem(bus.read(FLASH_XIP_BASE + addr + done, chunk, take));
    if (status == STATUS_OK) {
      crc = crc32_update(crc, chunk, take);
      done += (uint32_t)take;
    }
  }

  if (status != STATUS_OK) {
    return reply(response, CMD_RP_FLASH_CRC, status, BODY_LEN,
                 (uint32_t)REQUEST_LEN);
  }
  response[0] = CMD_RP_FLASH_CRC;
  response[1] = (uint8_t)status;
  store_le32(response + 2, crc ^ 0xFFFFFFFFU);
  return ((uint32_t)REQUEST_LEN << 16) | (uint32_t)(2U + BODY_LEN);
}

}  // namespace

bool handles(uint8_t cmd) {
  return (cmd >= CMD_RP_ATTACH) && (cmd <= CMD_RP_FLASH_CRC);
}

void reset() {
  session.attached = false;
}

uint32_t handle_command(const uint8_t *request, uint8_t *response,
                        size_t request_room) {
  const uint8_t cmd = request[0];

  // Bytes each command's fixed part occupies, command ID included. Reading a
  // field that is not in the packet would be reading whatever follows it.
  size_t need = 1U;
  switch (cmd) {
    case CMD_RP_ATTACH:        need = 15U; break;
    case CMD_RP_CORE:          need =  4U; break;
    case CMD_RP_CALL:          need = 22U; break;
    case CMD_RP_FLASH_PREP:    need =  1U; break;
    case CMD_RP_FLASH_ERASE:   need = 16U; break;
    case CMD_RP_FLASH_STAGE:   need =  7U; break;  // payload checked below
    case CMD_RP_FLASH_PROGRAM: need = 15U; break;
    case CMD_RP_FLASH_FINISH:  need =  2U; break;
    case CMD_RP_FLASH_CRC:     need =  9U; break;
    default:
      response[0] = 0xFFU;    // ID_DAP_Invalid
      return (1U << 16) | 1U;
  }

  if (request_room < need) {
    // Truncated. Claim what is left of the packet so a batch stops here
    // instead of carrying on from an offset past its end.
    return reply(response, cmd, STATUS_BAD_REQUEST, 0U, (uint32_t)request_room);
  }

  switch (cmd) {
    case CMD_RP_ATTACH:        return cmd_attach(request, response);
    case CMD_RP_CORE:          return cmd_core(request, response);
    case CMD_RP_CALL:          return cmd_call(request, response);
    case CMD_RP_FLASH_PREP:    return cmd_flash_prep(response);
    case CMD_RP_FLASH_ERASE:   return cmd_flash_erase(request, response);
    case CMD_RP_FLASH_STAGE:
      return cmd_flash_stage(request, response, request_room);
    case CMD_RP_FLASH_PROGRAM: return cmd_flash_program(request, response);
    case CMD_RP_FLASH_FINISH:  return cmd_flash_finish(request, response);
    default:                   return cmd_flash_crc(request, response);
  }
}

}  // namespace rp2040
