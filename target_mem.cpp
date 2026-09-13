/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * MEM-AP memory access for probe-side commands. See target_mem.h for what a
 * Burst is for and why it has to put the DP/AP registers back.
 */

#include "target_mem.h"

#include <string.h>

#include "logging.h"
#include "swd_port.h"

namespace target_mem {
namespace {

// Target words are little endian on the Cortex-M parts this talks to, so a
// word read from DRW maps to memory bytes lowest-first.
inline uint32_t load_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

inline void store_le32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

}  // namespace

const char *status_name(Status s) {
  switch (s) {
    case Status::ok:             return "ok";
    case Status::not_connected:  return "not_connected";
    case Status::sticky_error:   return "sticky_error";
    case Status::transfer_fault: return "transfer_fault";
    case Status::unsupported:    return "unsupported";
  }
  return "unknown";
}

// --- transfer helpers ------------------------------------------------------

Status Burst::dp_read(uint32_t reg, uint32_t *out) {
  uint32_t value = 0U;
  const uint8_t ack =
      swd::transfer(swd::REQ_RnW | (reg & swd::REQ_ADDR), &value);
  if (ack != swd::ACK_OK) {
    return Status::transfer_fault;
  }
  if (out != nullptr) {
    *out = value;
  }
  return Status::ok;
}

Status Burst::dp_write(uint32_t reg, uint32_t value) {
  uint32_t v = value;
  const uint8_t ack = swd::transfer(reg & swd::REQ_ADDR, &v);
  return (ack == swd::ACK_OK) ? Status::ok : Status::transfer_fault;
}

// An AP read is posted: the access starts the read, and the value comes back
// from the next AP read or from DP RDBUFF.
Status Burst::ap_read(uint32_t reg, uint32_t *out) {
  const uint8_t ack = swd::transfer(
      swd::REQ_APnDP | swd::REQ_RnW | (reg & swd::REQ_ADDR), nullptr);
  if (ack != swd::ACK_OK) {
    return Status::transfer_fault;
  }
  return dp_read(swd::DP_REG_RDBUFF, out);
}

Status Burst::ap_write(uint32_t reg, uint32_t value) {
  uint32_t v = value;
  const uint8_t ack =
      swd::transfer(swd::REQ_APnDP | (reg & swd::REQ_ADDR), &v);
  return (ack == swd::ACK_OK) ? Status::ok : Status::transfer_fault;
}

// --- register staging ------------------------------------------------------

Status Burst::select_ap(uint8_t ap) {
  // APSEL in the top byte, AP bank 0 so CSW/TAR/DRW are what A[3:2] names,
  // DP bank 0 so CTRL/STAT is readable. The client's own SELECT is restored
  // wholesale at the end, so narrowing it here costs nothing.
  const uint32_t value = (uint32_t)ap << 24;
  if (select_known_ && (select_ == value)) {
    return Status::ok;
  }

  const Status st = dp_write(swd::DP_REG_SELECT, value);
  if (st != Status::ok) {
    select_known_ = false;
    return st;
  }
  select_       = value;
  select_known_ = true;
  // A different AP has its own CSW and TAR, so what we knew about them no
  // longer describes the selected one.
  tar_known_ = false;
  csw_       = 0U;
  return Status::ok;
}

Status Burst::set_csw(uint32_t size, uint32_t addrinc) {
  const uint32_t value = csw_base_ | (size & swd::CSW_SIZE_MASK) |
                         (addrinc & swd::CSW_ADDRINC_MASK);
  if (value == csw_) {
    return Status::ok;
  }
  const Status st = ap_write(swd::AP_REG_CSW, value);
  if (st != Status::ok) {
    csw_ = 0U;
    return st;
  }
  csw_ = value;
  return Status::ok;
}

Status Burst::set_tar(uint32_t addr) {
  if (tar_known_ && (tar_ == addr)) {
    return Status::ok;
  }
  const Status st = ap_write(swd::AP_REG_TAR, addr);
  if (st != Status::ok) {
    tar_known_ = false;
    return st;
  }
  tar_       = addr;
  tar_known_ = true;
  return Status::ok;
}

// Byte-sized accesses are what let a write touch exactly the bytes it was
// asked to, instead of read-modify-writing a word the target may be changing
// underneath us. The size field is read-only for sizes an AP does not
// implement, so the only way to know is to write it and read it back.
Status Burst::ensure_byte_access() {
  if (byte_probed_) {
    return byte_ok_ ? Status::ok : Status::unsupported;
  }
  byte_probed_ = true;

  Status st = set_csw(swd::CSW_SIZE_BYTE, swd::CSW_ADDRINC_OFF);
  if (st != Status::ok) {
    return st;
  }

  uint32_t read_back = 0U;
  st = ap_read(swd::AP_REG_CSW, &read_back);
  if (st != Status::ok) {
    return st;
  }

  byte_ok_ = (read_back & swd::CSW_SIZE_MASK) == swd::CSW_SIZE_BYTE;
  if (!byte_ok_) {
    LOGW_KV("MEM-AP does not support byte accesses",
            "ap=%u csw=0x%08x", (unsigned)ap_, (unsigned)read_back);
  }
  return byte_ok_ ? Status::ok : Status::unsupported;
}

Status Burst::check_sticky(uint32_t *ctrl_stat) {
  uint32_t value = 0U;
  const Status st = dp_read(swd::DP_REG_CTRL_STAT, &value);
  if (st != Status::ok) {
    return st;
  }
  if (ctrl_stat != nullptr) {
    *ctrl_stat = value;
  }
  return ((value & swd::CTRL_STICKY_ANY) != 0U) ? Status::sticky_error
                                                : Status::ok;
}

// --- construction and teardown ---------------------------------------------

Burst::Burst(uint8_t ap)
    : ap_(ap),
      status_(Status::ok),
      restore_needed_(false),
      saved_(),
      select_known_(false),
      select_(0U),
      csw_base_(0U),
      csw_(0U),
      tar_(0U),
      tar_known_(false),
      byte_probed_(false),
      byte_ok_(false) {
  if (!swd::port_is_swd()) {
    status_ = Status::not_connected;
    return;
  }

  // From here on every transfer is ours, so stop the shadow following them,
  // and remember what it held: that is the state the client believes in.
  saved_ = swd::shadow();
  swd::snoop_suspend();
  restore_needed_ = true;

  status_ = select_ap(ap_);
  if (status_ != Status::ok) {
    return;
  }

  // Refuse to run on a DP that is already in an error state: we could not
  // tell our own faults from the ones that were there when we arrived, and
  // clearing the latter would hide a failure the client still has to see.
  status_ = check_sticky(nullptr);
  if (status_ != Status::ok) {
    return;
  }

  // Keep the client's bus attributes (HPROT, MasterType, and the rest) and
  // drive only the size and auto-increment fields.
  uint32_t csw = 0U;
  status_ = ap_read(swd::AP_REG_CSW, &csw);
  if (status_ != Status::ok) {
    return;
  }
  csw_base_ = csw & ~(swd::CSW_SIZE_MASK | swd::CSW_ADDRINC_MASK);
  csw_      = csw;
}

Burst::~Burst() {
  restore();
}

void Burst::restore() {
  if (!restore_needed_) {
    return;
  }
  restore_needed_ = false;

  // The constructor refused to start with a sticky error set, so any that is
  // set now is one of ours, and leaving it would fail the client's next
  // transfer for a reason the client could not account for.
  uint32_t ctrl = 0U;
  if ((dp_read(swd::DP_REG_CTRL_STAT, &ctrl) == Status::ok) &&
      ((ctrl & swd::CTRL_STICKY_ANY) != 0U)) {
    LOGD_KV("clearing sticky DP error left by a probe-side burst",
            "ap=%u ctrl_stat=0x%08x", (unsigned)ap_, (unsigned)ctrl);
    dp_write(swd::DP_REG_ABORT,
             swd::ABORT_STKCMPCLR | swd::ABORT_STKERRCLR |
             swd::ABORT_WDERRCLR | swd::ABORT_ORUNERRCLR);
  }

  // Put back what the client cached, in an order that leaves SELECT last:
  // reaching an AP's CSW or TAR means selecting that AP first.
  if (saved_.csw_valid && (select_ap(saved_.csw_ap) == Status::ok)) {
    ap_write(swd::AP_REG_CSW, saved_.csw);
  }
  if (saved_.tar_valid && (select_ap(saved_.tar_ap) == Status::ok)) {
    ap_write(swd::AP_REG_TAR, saved_.tar);
  }
  if (saved_.select_valid) {
    dp_write(swd::DP_REG_SELECT, saved_.select);
  }

  swd::snoop_resume();
}

// --- memory access ---------------------------------------------------------

Status Burst::rw_words(uint32_t addr, uint8_t *buf, size_t words,
                       bool is_write) {
  Status st = set_csw(swd::CSW_SIZE_WORD, swd::CSW_ADDRINC_SINGLE);
  if (st != Status::ok) {
    return st;
  }

  const uint32_t region = swd::TAR_AUTOINC_REGION;

  while (words != 0U) {
    // TAR only auto-increments within the 1kB region it is in, so take the
    // run up to the next boundary and reload TAR there.
    size_t chunk = (region - (addr & (region - 1U))) / 4U;
    if (chunk > words) {
      chunk = words;
    }

    st = set_tar(addr);
    if (st != Status::ok) {
      return st;
    }

    if (is_write) {
      for (size_t i = 0; i < chunk; i++) {
        st = ap_write(swd::AP_REG_DRW, load_le32(buf + 4U * i));
        if (st != Status::ok) {
          tar_known_ = false;
          return st;
        }
      }
    } else {
      // Post the first read, let each following read return the previous
      // one, and collect the last from RDBUFF: chunk+1 transfers for chunk
      // words instead of the 2*chunk a read-then-RDBUFF pair would cost.
      const uint32_t drw_read =
          swd::REQ_APnDP | swd::REQ_RnW | swd::AP_REG_DRW;
      if (swd::transfer(drw_read, nullptr) != swd::ACK_OK) {
        tar_known_ = false;
        return Status::transfer_fault;
      }
      for (size_t i = 0; i + 1U < chunk; i++) {
        uint32_t value = 0U;
        if (swd::transfer(drw_read, &value) != swd::ACK_OK) {
          tar_known_ = false;
          return Status::transfer_fault;
        }
        store_le32(buf + 4U * i, value);
      }
      uint32_t last = 0U;
      st = dp_read(swd::DP_REG_RDBUFF, &last);
      if (st != Status::ok) {
        tar_known_ = false;
        return st;
      }
      store_le32(buf + 4U * (chunk - 1U), last);
    }

    // Every DRW access moved TAR on by a word, wrapping inside the region
    // rather than carrying out of it -- which is exactly what happens when a
    // chunk ends on the boundary.
    const uint32_t end = addr + (uint32_t)(4U * chunk);
    tar_ = (addr & ~(region - 1U)) | (end & (region - 1U));

    addr  += (uint32_t)(4U * chunk);
    buf   += 4U * chunk;
    words -= chunk;
  }

  return Status::ok;
}

Status Burst::rw_bytes(uint32_t addr, uint8_t *buf, size_t len, bool is_write) {
  Status st = ensure_byte_access();
  if (st != Status::ok) {
    return st;
  }
  st = set_csw(swd::CSW_SIZE_BYTE, swd::CSW_ADDRINC_OFF);
  if (st != Status::ok) {
    return st;
  }

  for (size_t i = 0; i < len; i++) {
    const uint32_t at   = addr + (uint32_t)i;
    const unsigned lane = 8U * (at & 3U);   // byte lane within DRW

    st = set_tar(at);
    if (st != Status::ok) {
      return st;
    }

    if (is_write) {
      st = ap_write(swd::AP_REG_DRW, (uint32_t)buf[i] << lane);
      if (st != Status::ok) {
        return st;
      }
    } else {
      uint32_t value = 0U;
      st = ap_read(swd::AP_REG_DRW, &value);
      if (st != Status::ok) {
        return st;
      }
      buf[i] = (uint8_t)(value >> lane);
    }
  }

  return Status::ok;
}

Status Burst::read(uint32_t addr, void *dst, size_t len) {
  if (status_ != Status::ok) {
    return status_;
  }
  uint8_t *out = static_cast<uint8_t *>(dst);

  size_t head = (4U - (addr & 3U)) & 3U;      // bytes up to the next word
  if (head > len) {
    head = len;
  }
  if (head != 0U) {
    const Status st = rw_bytes(addr, out, head, false);
    if (st != Status::ok) {
      return st;
    }
    addr += (uint32_t)head;
    out  += head;
    len  -= head;
  }

  const size_t words = len / 4U;
  if (words != 0U) {
    const Status st = rw_words(addr, out, words, false);
    if (st != Status::ok) {
      return st;
    }
    addr += (uint32_t)(4U * words);
    out  += 4U * words;
    len  -= 4U * words;
  }

  if (len != 0U) {
    return rw_bytes(addr, out, len, false);
  }
  return Status::ok;
}

Status Burst::write(uint32_t addr, const void *src, size_t len) {
  if (status_ != Status::ok) {
    return status_;
  }
  // The AP layer only reads from this buffer; the const_cast keeps one
  // implementation for both directions.
  uint8_t *in = const_cast<uint8_t *>(static_cast<const uint8_t *>(src));

  size_t head = (4U - (addr & 3U)) & 3U;
  if (head > len) {
    head = len;
  }
  if (head != 0U) {
    const Status st = rw_bytes(addr, in, head, true);
    if (st != Status::ok) {
      return st;
    }
    addr += (uint32_t)head;
    in   += head;
    len  -= head;
  }

  const size_t words = len / 4U;
  if (words != 0U) {
    const Status st = rw_words(addr, in, words, true);
    if (st != Status::ok) {
      return st;
    }
    addr += (uint32_t)(4U * words);
    in   += 4U * words;
    len  -= 4U * words;
  }

  if (len != 0U) {
    const Status st = rw_bytes(addr, in, len, true);
    if (st != Status::ok) {
      return st;
    }
  }

  // An AP write completes after its transfer acks, so a write that faulted
  // shows up only as a sticky error. Anything set here is ours -- see the
  // constructor -- and restore() clears it.
  return check_sticky(nullptr);
}

Status Burst::clear_sticky() {
  return dp_write(swd::DP_REG_ABORT,
                  swd::ABORT_STKCMPCLR | swd::ABORT_STKERRCLR |
                  swd::ABORT_WDERRCLR | swd::ABORT_ORUNERRCLR);
}

Status Burst::read_u32(uint32_t addr, uint32_t *out) {
  uint8_t buf[4];
  const Status st = read(addr, buf, sizeof(buf));
  if (st != Status::ok) {
    return st;
  }
  *out = load_le32(buf);
  return Status::ok;
}

Status Burst::write_u32(uint32_t addr, uint32_t value) {
  uint8_t buf[4];
  store_le32(buf, value);
  return write(addr, buf, sizeof(buf));
}

}  // namespace target_mem
