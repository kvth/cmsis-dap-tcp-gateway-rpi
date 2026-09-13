// White-box test of the DP/AP shadow in DAP.cpp. Includes the translation unit
// so the file-static tracker and its state are reachable, with the GPIO
// registers pointed at a plain buffer instead of /dev/gpiomem.
#include <stdio.h>
#include <stdint.h>

#include "gpio_regs.h"

namespace gpio {
unsigned int swclk = 0, swdio_in = 1, swdio_out = 2, swdio_dir = 3;
bool swdio_single_pin = true, swdio_has_dir = false;
static uint32_t fake_regs[gpio::BLOCK_SIZE / 4];
volatile uint32_t *regs = fake_regs;
int swdio_is_output = 1;
bool is_mapped(void) { return true; }
void deinit(void) {}
void init_once(void) {}
}  // namespace gpio

namespace swdmux {
bool select_pos(unsigned int) { return true; }
bool is_enabled(void) { return false; }
int current_pos(void) { return -1; }
}  // namespace swdmux

#include "DAP.cpp"

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
    printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

static void dp_write(uint32_t reg, uint32_t v) { Shadow_Track(reg & 0x0C, &v); }
static void ap_write(uint32_t reg, uint32_t v) {
  Shadow_Track(DAP_TRANSFER_APnDP | (reg & 0x0C), &v);
}
static void ap_read(uint32_t reg) {
  uint32_t v = 0;
  Shadow_Track(DAP_TRANSFER_APnDP | DAP_TRANSFER_RnW | (reg & 0x0C), &v);
}

int main() {
  const uint32_t CSW  = 0x00, TAR = 0x04, DRW = 0x0C;
  const uint32_t SELECT = 0x08;

  // Nothing is trusted until the client writes it.
  swd::shadow_reset();
  ap_write(CSW, 0x23000052);
  CHECK(!swd::shadow().csw_valid, "CSW tracked without a SELECT");

  // A normal OpenOCD-shaped sequence: SELECT, CSW, TAR, then DRW bursts.
  swd::shadow_reset();
  dp_write(SELECT, 0x00000000);
  ap_write(CSW, 0x23000052);           // word size, single auto-increment
  ap_write(TAR, 0x20000000);
  CHECK(swd::shadow().select_valid && swd::shadow().select == 0, "SELECT");
  CHECK(swd::shadow().csw_valid && swd::shadow().csw == 0x23000052, "CSW");
  CHECK(swd::shadow().tar_valid && swd::shadow().tar == 0x20000000, "TAR");

  for (int i = 0; i < 4; i++) ap_read(DRW);
  CHECK(swd::shadow().tar == 0x20000010, "TAR after 4 words = 0x%08x",
        swd::shadow().tar);

  // Byte-sized accesses move TAR by one.
  ap_write(CSW, 0x23000050);           // byte size, single auto-increment
  for (int i = 0; i < 3; i++) ap_read(DRW);
  CHECK(swd::shadow().tar == 0x20000013, "TAR after 3 bytes = 0x%08x",
        swd::shadow().tar);

  // Auto-increment off means TAR stays put.
  ap_write(CSW, 0x23000002);
  ap_read(DRW);
  CHECK(swd::shadow().tar == 0x20000013, "TAR with auto-increment off = 0x%08x",
        swd::shadow().tar);

  // Increment wraps inside the 1kB region rather than carrying out of it.
  ap_write(CSW, 0x23000052);
  ap_write(TAR, 0x200003FC);
  ap_read(DRW);
  CHECK(swd::shadow().tar == 0x20000000, "TAR at the region boundary = 0x%08x",
        swd::shadow().tar);

  // Writes advance TAR the same way reads do.
  ap_write(TAR, 0x20001000);
  { uint32_t v = 0xDEADBEEF; ap_write(DRW, v); }
  CHECK(swd::shadow().tar == 0x20001004, "TAR after a DRW write = 0x%08x",
        swd::shadow().tar);

  // Accesses under a non-zero AP bank are not CSW/TAR/DRW.
  dp_write(SELECT, 0x000000F0);
  ap_write(CSW, 0x11111111);
  CHECK(swd::shadow().csw == 0x23000052, "CSW clobbered from bank 0xF");

  // Each AP has its own registers.
  dp_write(SELECT, 0x01000000);
  ap_write(TAR, 0xE000E000);
  CHECK(swd::shadow().tar_ap == 1 && swd::shadow().tar == 0xE000E000, "AP 1 TAR");
  CHECK(swd::shadow().csw_ap == 0, "CSW should still belong to AP 0");
  ap_read(DRW);   // CSW belongs to another AP, so no increment is modelled
  CHECK(swd::shadow().tar == 0xE000E000, "TAR moved on an AP with no known CSW");

  // Suspending tracking is what keeps a probe-side burst out of the shadow.
  dp_write(SELECT, 0x00000000);
  ap_write(TAR, 0x20002000);
  swd::snoop_suspend();
  ap_write(TAR, 0x08000000);
  dp_write(SELECT, 0x0F000000);
  swd::snoop_resume();
  CHECK(swd::shadow().tar == 0x20002000, "TAR tracked while suspended");
  CHECK(swd::shadow().select == 0x00000000, "SELECT tracked while suspended");

  // --- vendor commands through the real dispatch ---
  {
    uint8_t resp[2048];

    // Vendor0 consumes its position byte, so a batch after it stays in step.
    const uint8_t mux[] = {0x80, 0x01};
    uint32_t n = cmsis::dap::process_cmd(mux, resp);
    CHECK((n >> 16) == 2, "Vendor0 request length %u", n >> 16);
    CHECK((n & 0xFFFF) == 2, "Vendor0 response length %u", n & 0xFFFF);

    // An RTT command with the port down fails cleanly rather than bitbanging.
    const uint8_t start[] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x20,
                             0x00, 0x00, 0x00, 0x00};
    n = cmsis::dap::process_cmd(start, resp);
    CHECK((n >> 16) == 10, "RTT_Start request length %u", n >> 16);
    CHECK((n & 0xFFFF) == 8, "RTT_Start response length %u", n & 0xFFFF);
    CHECK(resp[0] == 0x81 && resp[1] == vendor::STATUS_NOT_CONNECTED,
          "RTT_Start with the port down: id %u status %u", resp[0], resp[1]);

    // An unclaimed vendor ID is still rejected.
    const uint8_t unused[] = {0x9F};
    n = cmsis::dap::process_cmd(unused, resp);
    CHECK(resp[0] == ID_DAP_Invalid, "unused vendor ID answered %u", resp[0]);

    // A batch: mux select, RTT_Stop, RTT_Write of 3 bytes, RTT_Read. Each
    // response has to land where the previous one ended, which only works if
    // every handler reports the request length it actually consumed.
    const uint8_t batch[] = {
        0x7F, 0x04,
        0x80, 0x01,
        0x82,
        0x85, 0x00, 0x03, 0x00, 'a', 'b', 'c',
        0x84, 0x00, 0x10, 0x00,
    };
    n = cmsis::dap::execute_cmd(batch, resp);
    CHECK((n >> 16) == sizeof(batch), "batch request length %u vs %zu",
          n >> 16, sizeof(batch));
    CHECK((n & 0xFFFF) == 2 + 2 + 2 + 4 + 4, "batch response length %u",
          n & 0xFFFF);
    CHECK(resp[0] == 0x7F && resp[1] == 4, "batch header");
    CHECK(resp[2] == 0x80, "batch: mux response at 2, got %u", resp[2]);
    CHECK(resp[4] == 0x82 && resp[5] == vendor::STATUS_OK,
          "batch: RTT_Stop response at 4, got %u/%u", resp[4], resp[5]);
    CHECK(resp[6] == 0x85 && resp[7] == vendor::STATUS_NOT_STARTED,
          "batch: RTT_Write response at 6, got %u/%u", resp[6], resp[7]);
    CHECK(resp[10] == 0x84 && resp[11] == vendor::STATUS_NOT_STARTED,
          "batch: RTT_Read response at 10, got %u/%u", resp[10], resp[11]);
  }

  printf(failures ? "\n%d FAILURES\n" : "\nall checks passed\n", failures);
  return failures != 0;
}
