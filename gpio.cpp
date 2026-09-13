#include "gpio_regs.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/mman.h>

#include "logging.h"

namespace gpio {

namespace {

// Highest GPIO number addressable through the BCM283x/BCM2711 register block.
constexpr unsigned int MAX_PIN = 53;

int mem_fd = -1;

struct Pin {
    const char *name;
    unsigned int number;
};

// Reject a mapping this hardware cannot drive, or one that puts two signals on
// a single GPIO. Either wiring cannot work, and left unchecked they surface as
// confusing SWD protocol errors much later instead of a startup failure.
bool pins_usable(const Pin *pins, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (pins[i].number > MAX_PIN) {
            LOGE_KV("gpio number out of range",
                    "signal=%s pin=%u max=%u", pins[i].name, pins[i].number, MAX_PIN);
            return false;
        }
        for (size_t j = i + 1; j < n; j++) {
            if (pins[i].number == pins[j].number) {
                LOGE_KV("two signals on one gpio",
                        "signal=%s other=%s pin=%u",
                        pins[i].name, pins[j].name, pins[i].number);
                return false;
            }
        }
    }
    return true;
}

}  // namespace

// Set by configure_split() / configure_single() before any access, read-only
// from then on, so the bitbang paths can treat them as constants.
unsigned int swclk     = 0;
unsigned int swdio_in  = 0;
unsigned int swdio_out = 0;
unsigned int swdio_dir = 0;
bool swdio_single_pin  = false;
bool swdio_has_dir     = false;

volatile uint32_t *regs = NULL;
int swdio_is_output = 0;

bool configure_split(unsigned int swclk_pin, unsigned int swdio_in_pin,
                     unsigned int swdio_out_pin, unsigned int swdio_dir_pin)
{
    const Pin pins[] = {
        { "swclk",     swclk_pin     },
        { "swdio_in",  swdio_in_pin  },
        { "swdio_out", swdio_out_pin },
        { "swdio_dir", swdio_dir_pin },
    };
    if (!pins_usable(pins, sizeof(pins) / sizeof(pins[0]))) {
        return false;
    }

    swclk     = swclk_pin;
    swdio_in  = swdio_in_pin;
    swdio_out = swdio_out_pin;
    swdio_dir = swdio_dir_pin;
    swdio_single_pin = false;
    swdio_has_dir    = true;
    return true;
}

bool configure_single(unsigned int swclk_pin, unsigned int swdio_pin,
                      bool has_dir, unsigned int swdio_dir_pin)
{
    const Pin pins[] = {
        { "swclk",     swclk_pin     },
        { "swdio",     swdio_pin     },
        { "swdio_dir", swdio_dir_pin },
    };
    if (!pins_usable(pins, has_dir ? 3 : 2)) {
        return false;
    }

    swclk = swclk_pin;
    // One pin for both directions: the in and out pin numbers are the same and
    // request_swdio_*() flips the GPIO itself.
    swdio_in = swdio_out = swdio_pin;
    swdio_dir = swdio_dir_pin;
    swdio_single_pin = true;
    swdio_has_dir    = has_dir;
    return true;
}

void init_once(void)
{
    if (regs != NULL) {
        return;
    }

    mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        // Usually one of: not running as root / not in the gpio group, or a
        // board without this interface at all (a Pi 5 or CM5 exposes
        // /dev/gpiomem0..4 with an incompatible RP1 layout, not /dev/gpiomem).
        LOGE_KV("open /dev/gpiomem failed", "err=\"%s\"", strerror(errno));
        LOGE("need root (or the gpio group) on a Pi 1-4 / CM1, CM3, CM4");
        exit(1);
    }

    regs = (volatile uint32_t *)mmap(NULL, BLOCK_SIZE,
                                     PROT_READ | PROT_WRITE,
                                     MAP_SHARED,
                                     mem_fd,
                                     0);
    if (regs == MAP_FAILED) {
        LOGE_KV("mmap /dev/gpiomem failed", "err=\"%s\"", strerror(errno));
        regs = NULL;
        close(mem_fd);
        mem_fd = -1;
        exit(1);
    }
}

void deinit(void)
{
    if (regs != NULL) {
        munmap((void *)regs, BLOCK_SIZE);
        regs = NULL;
    }
    if (mem_fd >= 0) {
        close(mem_fd);
        mem_fd = -1;
    }
}

bool is_mapped(void)
{
    return regs != NULL;
}

}  // namespace gpio
