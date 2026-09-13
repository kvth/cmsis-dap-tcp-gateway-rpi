#ifndef GPIO_REGS_H
#define GPIO_REGS_H

// The BCM283x/BCM2711 register-poking layer behind gpio.h.
//
// This is a header full of inline functions and mutable globals only because
// the SWD bitbang loop calls it once per clock edge and needs it inlined.
// Include it from the pin abstraction (DAP_config.h) and gpio.cpp; everything
// else talks to the GPIO through gpio.h instead.
//
// Everything here is in namespace gpio, and everything here is written only by
// gpio::configure_split() / gpio::configure_single() at startup.

#include <stddef.h>
#include <stdint.h>

namespace gpio {

// Resolved pin mapping. In single pin mode swdio_in == swdio_out, and
// swdio_dir is only touched when swdio_has_dir says one is wired.
extern unsigned int swclk;
extern unsigned int swdio_in;
extern unsigned int swdio_out;
extern unsigned int swdio_dir;
extern bool swdio_single_pin;
extern bool swdio_has_dir;

// Mapped GPIO register block and the current SWDIO direction.
extern volatile uint32_t *regs;
extern int swdio_is_output;

// BCM2711 / Raspberry Pi GPIO register offsets, in bytes from the GPIO base.
inline constexpr unsigned int GPFSEL0 = 0x00;   // function select (3 bits/pin)
inline constexpr unsigned int GPSET0  = 0x1C;   // write 1 to drive a pin high
inline constexpr unsigned int GPCLR0  = 0x28;   // write 1 to drive a pin low
inline constexpr unsigned int GPLEV0  = 0x34;   // read pin levels

inline constexpr size_t BLOCK_SIZE = 4096;

// GPFSEL function codes.
inline constexpr uint32_t FUNC_INPUT  = 0;
inline constexpr uint32_t FUNC_OUTPUT = 1;

static inline volatile uint32_t *reg(unsigned int byte_offset)
{
    return (volatile uint32_t *)((volatile uint8_t *)regs + byte_offset);
}

// Flush pending GPIO register writes out of the inner shareable domain, so two
// writes to the same address cannot be merged into one and lose an edge.
// `volatile` only stops the *compiler* reordering or eliding them; this stops
// the CPU's write buffer doing it. OpenOCD does the same thing around every
// register access in its own Pi bitbang driver
// (bcm2835_gpio_synchronize() in src/jtag/drivers/bcm2835gpio.c), and its
// published speed coefficients were measured with these barriers in place.
static inline void synchronize(void)
{
    __sync_synchronize();
}

static inline void write_mask(unsigned int byte_offset, unsigned int pin)
{
    reg(byte_offset)[pin / 32] = (1u << (pin % 32));
    synchronize();
}

static inline uint32_t read_level(unsigned int pin)
{
    synchronize();
    return (reg(GPLEV0)[pin / 32] >> (pin % 32)) & 1u;
}

static inline void set_high(unsigned int pin)
{
    write_mask(GPSET0, pin);
}

static inline void set_low(unsigned int pin)
{
    write_mask(GPCLR0, pin);
}

static inline void set_function(unsigned int pin, unsigned int func)
{
    volatile uint32_t *fsel = reg(GPFSEL0) + (pin / 10);
    unsigned int shift = (pin % 10) * 3;
    uint32_t v = *fsel;
    v &= ~(7u << shift);
    v |= ((func & 7u) << shift);
    *fsel = v;
    synchronize();
}

static inline void set_input(unsigned int pin)
{
    set_function(pin, FUNC_INPUT);
}

static inline void set_output(unsigned int pin)
{
    set_function(pin, FUNC_OUTPUT);
}

// Switch a pin to output and drive it to `value` in one step.
static inline void drive_output(unsigned int pin, int value)
{
    set_output(pin);
    if (value) {
        set_high(pin);
    } else {
        set_low(pin);
    }
}

// Turn the SWDIO line around. The buffer is always pointed the right way
// before the pin itself starts driving (and stops driving before the buffer
// turns back around), so the two never fight over the line in between.
static inline void request_swdio_output(void)
{
    if (swdio_has_dir) {
        set_low(swdio_dir);   // 0 = host drives target
    }
    if (swdio_single_pin) {
        set_output(swdio_out);
    }
    swdio_is_output = 1;
}

static inline void request_swdio_input(void)
{
    if (swdio_single_pin) {
        set_input(swdio_in);
    }
    if (swdio_has_dir) {
        set_high(swdio_dir);  // 1 = host reads from target
    }
    swdio_is_output = 0;
}

}  // namespace gpio

#endif  // GPIO_REGS_H
