#ifndef GPIO_H
#define GPIO_H

// Raspberry Pi GPIO access: a direct mmap() of /dev/gpiomem, poking the
// BCM283x/BCM2711 GPFSEL/GPSET/GPCLR/GPLEV registers. That register layout is
// shared by the Pi 1 through Pi 4 (and CM1/CM3/CM4), but NOT by the Pi 5 / CM5,
// whose RP1 southbridge has a different layout and no legacy /dev/gpiomem.
//
// The register poking itself lives in gpio_regs.h, which is inlined into the
// SWD bitbang path; this header is the rest of the world's view of the GPIO.
//
// SWDIO comes in two wirings, picked at startup:
//
//   split   separate input and output pins behind a buffer, whose direction is
//           driven by a third pin:
//             0 -> host drives target (use swdio_out)
//             1 -> host reads from target (use swdio_in)
//   single  one bidirectional pin wired straight to the target's SWDIO,
//           flipped between input and output in GPFSEL. A buffer direction pin
//           may still be wired alongside it, which is why the two are
//           independent.

namespace gpio {

// Pin assignment, in BCM numbering. Exactly one of these is called once at
// startup, before any other call here. Both reject a pin number this hardware
// cannot address and a mapping that reuses one GPIO for two signals, logging
// which pins were at fault.
//
//   configure_split():  separate SWDIO input and output pins behind a buffer
//                       whose direction is driven on swdio_dir.
//   configure_single(): one bidirectional SWDIO pin. swdio_dir is ignored, and
//                       no direction pin is driven, when has_dir is false.
bool configure_split(unsigned int swclk, unsigned int swdio_in,
                     unsigned int swdio_out, unsigned int swdio_dir);
bool configure_single(unsigned int swclk, unsigned int swdio,
                      bool has_dir, unsigned int swdio_dir);

// Open and mmap /dev/gpiomem. Exits the process on failure. Idempotent.
void init_once(void);

// Unmap the register block and close /dev/gpiomem. Safe to call even if
// init_once() was never called.
void deinit(void);

// Whether the register block is currently mapped.
bool is_mapped(void);

}  // namespace gpio

#endif  // GPIO_H
