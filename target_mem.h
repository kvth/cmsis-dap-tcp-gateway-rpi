/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Probe-side target memory access through a MEM-AP.
 *
 * The CMSIS-DAP commands the client sends are transfer-level: the client
 * decides which AP registers to poke and when. A probe-side feature like the
 * RTT vendor commands (see rtt.h) instead needs to read and write target
 * memory on its own, which means driving DP SELECT and the AP's CSW and TAR
 * itself -- registers the client caches.
 *
 * Burst is the bracket that makes that safe. Constructing one snapshots the
 * client's cached register values (see swd_port.h), and destroying one puts
 * the hardware back the way the client left it, including clearing any sticky
 * DP error our own transfers caused. Everything between is ordinary memory
 * access: unaligned addresses and lengths are handled with byte-sized AP
 * accesses so a write never has to read-modify-write memory the target is
 * concurrently using.
 *
 * A Burst is meant to live for one command, not to be held across commands:
 * while one exists the client's registers are not in the state the client
 * expects.
 */

#ifndef TARGET_MEM_H
#define TARGET_MEM_H

#include <stddef.h>
#include <stdint.h>

#include "swd_port.h"

namespace target_mem {

enum class Status : uint8_t {
  ok = 0,
  not_connected,      // the client has not put the debug port in SWD mode
  sticky_error,       // the DP already had a sticky error when we started
  transfer_fault,     // a DP or AP transfer failed
  unsupported,        // the AP cannot do an access we need (byte size)
};

const char *status_name(Status s);

class Burst {
 public:
  // Takes over the DP/AP registers for the duration. Check status() before
  // using the burst: on anything but ok the accessors will fail too.
  explicit Burst(uint8_t ap);
  ~Burst();

  Burst(const Burst &) = delete;
  Burst &operator=(const Burst &) = delete;

  Status status() const { return status_; }

  // Read/write target memory. Any address and length; the fast path is the
  // word-aligned middle, with byte accesses at the ends.
  Status read(uint32_t addr, void *dst, size_t len);
  Status write(uint32_t addr, const void *src, size_t len);

  // Convenience wrappers for the fixed-layout parts of a control block.
  Status read_u32(uint32_t addr, uint32_t *out);
  Status write_u32(uint32_t addr, uint32_t value);

  // Clear a sticky DP error so the burst can carry on. Only for a caller that
  // expects faults and means to continue past them -- scanning memory for a
  // control block walks over whatever is unmapped in the range it was given.
  Status clear_sticky();

 private:
  Status dp_read(uint32_t reg, uint32_t *out);
  Status dp_write(uint32_t reg, uint32_t value);
  Status ap_read(uint32_t reg, uint32_t *out);
  Status ap_write(uint32_t reg, uint32_t value);

  Status select_ap(uint8_t ap);
  Status set_csw(uint32_t size, uint32_t addrinc);
  Status set_tar(uint32_t addr);
  Status ensure_byte_access();
  Status check_sticky(uint32_t *ctrl_stat);

  Status rw_words(uint32_t addr, uint8_t *buf, size_t words, bool is_write);
  Status rw_bytes(uint32_t addr, uint8_t *buf, size_t len, bool is_write);

  void restore();

  uint8_t     ap_;
  Status      status_;
  bool        restore_needed_;    // we touched the hardware and must undo it
  swd::Shadow saved_;

  bool        select_known_;      // whether select_ describes the hardware
  uint32_t    select_;            // full SELECT value we last wrote

  uint32_t    csw_base_;          // the client's CSW with size/addrinc cleared
  uint32_t    csw_;               // CSW value we last wrote
  uint32_t    tar_;               // TAR as the AP has it now
  bool        tar_known_;

  bool        byte_probed_;
  bool        byte_ok_;
};

}  // namespace target_mem

#endif  // TARGET_MEM_H
