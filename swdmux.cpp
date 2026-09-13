#include "swdmux.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include "logging.h"

namespace swdmux {

namespace {

constexpr uint8_t SWDMUX_REG_IODIR   = 0x06;  // direction: select lines (bits 0-3) out, rest in
constexpr uint8_t SWDMUX_REG_GPIO    = 0x02;  // output level of the select lines
constexpr uint8_t SWDMUX_IODIR_VALUE = 0xF0;  // 0b11110000: bits 0-3 output, bits 4-7 input
constexpr int SWDMUX_MAX_RETRIES     = 3;


#ifndef I2C_SLAVE
#define I2C_SLAVE 0x0703    // from <linux/i2c-dev.h>; hardcoded to avoid the header dependency
#endif

// 0bDCBA, one 4-bit select code per logical node position 1..12. Identical to
// lookupTableSwdNodeSelect in swdmux.go.
const uint8_t SWDMUX_NODE_SELECT[12] = {
    0b1011, // New Node 1  (Old Node 11)
    0b1111, // New Node 2  (Old Node 12)
    0b0011, // New Node 3  (Old Node 9)
    0b0111, // New Node 4  (Old Node 10)
    0b1010, // New Node 5  (Old Node 7)
    0b1110, // New Node 6  (Old Node 8)
    0b0010, // New Node 7  (Old Node 5)
    0b0110, // New Node 8  (Old Node 6)
    0b1001, // New Node 9  (Old Node 3)
    0b1101, // New Node 10 (Old Node 4)
    0b0001, // New Node 11 (Old Node 1)
    0b0101, // New Node 12 (Old Node 2)
};

// 0bXX00 with the select bits zeroed selects the extender; any combination
// of the top two (unused) bits works, matching lookupExtender in swdmux.go.
const uint8_t SWDMUX_EXTENDER_SELECT = 0b0000;

// Set by init() before anything else runs, read-only from then on.
const char *g_i2c_dev = NULL;
unsigned int g_i2c_addr = 0;
bool g_enabled = false;

// Currently selected SWD mux position, or -1 if none has been selected yet
// (mux left untouched since process start). Updated on every successful
// swdmux_select_pos() and surfaced to the outside world by DAP_Info() under
// DAP_ID_DAP_FW_VER -- see the comment there for why that field was picked.
volatile int g_current_pos = -1;

bool i2c_write(int fd, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    ssize_t n = write(fd, buf, sizeof(buf));
    if (n != (ssize_t)sizeof(buf)) {
        LOGE_KV("i2c register write failed",
                "reg=0x%02x err=\"%s\"", reg, strerror(errno));
        return false;
    }
    return true;
}

bool select_pos_once(unsigned int pos)
{
    int fd = open(g_i2c_dev, O_RDWR);
    if (fd < 0) {
        LOGE_KV("open i2c device failed",
                "dev=%s err=\"%s\"", g_i2c_dev, strerror(errno));
        return false;
    }

    bool ok = false;
    if (ioctl(fd, I2C_SLAVE, g_i2c_addr) < 0) {
        LOGE_KV("i2c slave address select failed",
                "addr=0x%02x err=\"%s\"", g_i2c_addr, strerror(errno));
    } else {
        uint8_t sel = (pos == 0) ? SWDMUX_EXTENDER_SELECT : SWDMUX_NODE_SELECT[pos - 1];

        // Set the four select lines as outputs (the other four are unused).
        ok = i2c_write(fd, SWDMUX_REG_IODIR, SWDMUX_IODIR_VALUE);
        // Drive the select lines to the position's code.
        ok = ok && i2c_write(fd, SWDMUX_REG_GPIO, sel);
    }

    close(fd);
    return ok;
}

// Select logical SWD mux position: 0 = extender, 1..12 = node. Retries up to
// SWDMUX_MAX_RETRIES times on I2C errors, matching swdmux.go's SelectPos().
// Callable both at startup (argv[3]) and at runtime via
// DAP_ProcessVendorCommand(Vendor0).
}  // namespace

void init(const char *i2c_dev, unsigned int i2c_addr, bool enabled)
{
    g_i2c_dev  = i2c_dev;
    g_i2c_addr = i2c_addr;
    g_enabled  = enabled;
}

bool is_enabled(void)
{
    return g_enabled;
}

bool select_pos(unsigned int pos)
{
    if (!g_enabled) {
        LOGE("mux support is disabled (--no-swd-mux)");
        return false;
    }

    if (pos > 12) {
        LOGE_KV("invalid mux position", "pos=%u max=12", pos);
        return false;
    }

    for (int attempt = 1; attempt <= SWDMUX_MAX_RETRIES; attempt++) {
        if (select_pos_once(pos)) {
            LOGI_KV("selected mux position", "pos=%u", pos);
            g_current_pos = (int)pos;
            return true;
        }
        LOGW_KV("select mux position failed, retrying",
                "pos=%u attempt=%d retries=%d", pos, attempt, SWDMUX_MAX_RETRIES);
    }

    LOGE_KV("select mux position failed",
            "pos=%u attempts=%d", pos, SWDMUX_MAX_RETRIES);
    return false;
}

int current_pos(void)
{
    return g_current_pos;
}

}  // namespace swdmux
