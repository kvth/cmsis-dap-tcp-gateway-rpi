/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bitbanged SWD <-> CMSIS-DAP TCP proxy for the Raspberry Pi: command line
 * handling and startup.
 *
 * Adapted from https://github.com/bkuschak/cmsis_dap_tcp_esp32/, which serves
 * the same OpenOCD "cmsis-dap backend tcp" protocol from an ESP32 over WiFi.
 * That project in turn took its CMSIS-DAP core from
 * https://github.com/ARM-software/CMSIS-DAP (v2.1.2). Here the ESP-IDF/FreeRTOS
 * side is replaced by a plain Linux process and the ESP32 GPIO driver by an
 * mmap of the BCM283x/BCM2711 registers; the TCP wire format is unchanged.
 *
 * The pieces it wires together:
 *
 *   gpio.h/.cpp        mmap'ed BCM283x/BCM2711 GPIO access
 *   gpio_regs.h        the register poking inlined into the bitbang loop
 *   DAP_config.h       CMSIS-DAP I/O pin abstraction on top of gpio.h
 *   DAP.h/.cpp         CMSIS-DAP command processing and the SWD bitbang
 *   swdmux.h/.cpp      I2C GPIO expander that routes SWD to one connector
 *   tcp_server.h/.cpp  CMSIS-DAP over TCP framing and the server loop
 *
 * Usage:
 * # run gpio swd <-> cmsis dap tcp proxy (see --help for the full flag list)
 * sudo ./cmsis_dap_tcp --bind 127.0.0.1 --port 4441 --swd-pos 1
 *
 * # same, on a board with different wiring and no SWD mux
 * sudo ./cmsis_dap_tcp --bind 127.0.0.1 --port 4441 --no-swd-mux \
 *     --swclk 16 --swdio-in 12 --swdio-out 20 --swdio-dir 21
 *
 * # switch swd position to 0x01 node1 via ID_DAP_Vendor0 cmd (a different port
 * here just to show it does not have to match the examples above)
 * ./openocd -c "gdb_port disabled" -c "telnet_port disabled" -c "tcl_port disabled" -c "adapter driver cmsis-dap" -c "cmsis-dap backend tcp" -c "cmsis-dap tcp host 127.0.0.1" -c "cmsis-dap tcp port 4447" -c "transport select swd" -c "init; cmsis-dap cmd 0x80 0x01; cmsis-dap info; shutdown"
 *
 * # just test if swd device is there on position # Info : SWD DPIDR 0x0bc12477, DLPIDR 0x00000001
 * ./openocd -s scripts -c "gdb_port disabled" -c "telnet_port disabled" -c "tcl_port disabled" -c "adapter driver cmsis-dap" -c "cmsis-dap backend tcp" -c "cmsis-dap tcp host 127.0.0.1" -c "cmsis-dap tcp port 4447" -f target/rp2040.cfg -c "init; shutdown"
 */

#include <errno.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>

#include "DAP.h"
#include "gpio.h"
#include "logging.h"
#include "swdmux.h"
#include "tcp_server.h"

namespace {

// Defaults for everything the command line can set. They live here, with the
// flags that override them, rather than in the modules: each module is told
// what to use through its init()/configure() call.
constexpr int PORT_DEFAULT = 4441;
constexpr logging::Level LOG_LEVEL_DEFAULT = logging::Level::info;

constexpr const char *I2C_DEV_DEFAULT = "/dev/i2c-22";
constexpr unsigned int I2C_ADDR_DEFAULT = 0x20;

// Pin mapping (BCM numbering) of the carrier board this was written for.
constexpr unsigned int SWCLK_DEFAULT     = 16;
constexpr unsigned int SWDIO_IN_DEFAULT  = 12;
constexpr unsigned int SWDIO_OUT_DEFAULT = 20;
constexpr unsigned int SWDIO_DIR_DEFAULT = 21;

// Upper bound for pin arguments at parse time. What this hardware can actually
// address is gpio::configure_split()/configure_single()'s business, and they
// reject anything higher with the real limit in the message.
constexpr long PIN_ARG_MAX = 255;

// Everything the command line can set, in one place. parse_args() fills this
// in and main() hands each part to the module that owns it; nothing here is
// written straight into another module's state.
//
// The string members point into argv, which outlives every use of them, so
// nothing here owns or copies a string.
struct Options {
    // Server.
    const char *bind_addr = NULL;   // NULL = dual-stack wildcard
    int port = PORT_DEFAULT;
    // Non-NULL/non-empty: listen on this Unix domain socket instead of
    // bind_addr:port, for a client on the same machine. See tcp_server.h.
    const char *unix_socket = NULL;
    logging::Level log_level = LOG_LEVEL_DEFAULT;

    // SWD mux.
    const char *i2c_dev = I2C_DEV_DEFAULT;
    unsigned int i2c_addr = I2C_ADDR_DEFAULT;
    bool mux_enabled = true;
    bool has_swd_pos = false;   // whether a startup mux position was given
    unsigned int swd_pos = 0;   // 0 = extender, 1..12 = node

    // Pins (BCM numbering).
    unsigned int swclk = SWCLK_DEFAULT;
    unsigned int swdio_in = SWDIO_IN_DEFAULT;
    unsigned int swdio_out = SWDIO_OUT_DEFAULT;
    unsigned int swdio_dir = SWDIO_DIR_DEFAULT;
    bool swdio_single = false;  // --swdio: one bidirectional pin
    bool swdio_has_dir = true;  // whether a buffer direction pin is wired

    // SWD clock calibration; see cmsis::dap::SPEED_COEFF_DEFAULT.
    unsigned int speed_coeff = cmsis::dap::SPEED_COEFF_DEFAULT;
    unsigned int speed_offset = cmsis::dap::SPEED_OFFSET_DEFAULT;
    // Re-measure speed_coeff/speed_offset on this machine at startup instead
    // of trusting the two above; see cmsis::dap::calibrate().
    bool auto_calibrate = false;

    // Startup SWD clock, in Hertz; see cmsis::dap::CLOCK_KHZ_DEFAULT.
    unsigned int clock_hz = cmsis::dap::CLOCK_KHZ_DEFAULT * 1000U;
};

// Long-only options get values above the ASCII range so they cannot collide
// with the short options.
enum {
    OPT_SWCLK = 256,
    OPT_SWDIO,
    OPT_SWDIO_IN,
    OPT_SWDIO_OUT,
    OPT_SWDIO_DIR,
    OPT_I2C_DEV,
    OPT_I2C_ADDR,
    OPT_SPEED_COEFF,
    OPT_SPEED_OFFSET,
    OPT_AUTO_CALIBRATE,
    OPT_CLOCK,
    OPT_UNIX_SOCKET,
};

static const struct option LONG_OPTS[] = {
    { "bind",       required_argument, NULL, 'b' },
    { "port",       required_argument, NULL, 'p' },
    { "swd-pos",    required_argument, NULL, 's' },
    { "no-swd-mux", no_argument,       NULL, 'M' },
    { "swclk",      required_argument, NULL, OPT_SWCLK },
    { "swdio",      required_argument, NULL, OPT_SWDIO },
    { "swdio-in",   required_argument, NULL, OPT_SWDIO_IN },
    { "swdio-out",  required_argument, NULL, OPT_SWDIO_OUT },
    { "swdio-dir",  required_argument, NULL, OPT_SWDIO_DIR },
    { "i2c-dev",    required_argument, NULL, OPT_I2C_DEV },
    { "i2c-addr",   required_argument, NULL, OPT_I2C_ADDR },
    { "speed-coeff",  required_argument, NULL, OPT_SPEED_COEFF },
    { "speed-offset", required_argument, NULL, OPT_SPEED_OFFSET },
    { "auto-calibrate", no_argument,     NULL, OPT_AUTO_CALIBRATE },
    { "clock",      required_argument, NULL, OPT_CLOCK },
    { "unix-socket", required_argument, NULL, OPT_UNIX_SOCKET },
    { "log-level",  required_argument, NULL, 'l' },
    { "help",       no_argument,       NULL, 'h' },
    { NULL,         0,                 NULL, 0   },
};

static void print_usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s [options] [bind_address] [port] [swd_mux_position]\n"
        "\n"
        "Bitbanged SWD <-> CMSIS-DAP TCP proxy. Drives SWCLK/SWDIO over Raspberry Pi\n"
        "GPIO (BCM283x/BCM2711 register layout, i.e. Pi 1-4 and CM1/CM3/CM4).\n"
        "\n"
        "Options:\n"
        "  -b, --bind ADDR      address to listen on (default: all interfaces)\n"
        "  -p, --port PORT      TCP port to listen on (default: %d)\n"
        "      --unix-socket PATH  listen on this Unix domain socket instead of TCP,\n"
        "                       for a client on the same machine (skips the TCP/IP\n"
        "                       stack entirely -- not reachable from OpenOCD, which\n"
        "                       needs a real network endpoint)\n"
        "  -s, --swd-pos N      SWD mux position to select at startup: 0 = extender,\n"
        "                       1..12 = node (default: leave the mux untouched)\n"
        "  -M, --no-swd-mux     no SWD mux on this board: skip the startup select and\n"
        "                       reject the DAP_Vendor0 select command\n"
        "      --i2c-dev PATH   I2C bus the SWD mux sits on   (default: %s)\n"
        "      --i2c-addr ADDR  I2C address of the mux expander (default: 0x%02x)\n"
        "  -l, --log-level LVL  error, warn, info or debug (default: %s)\n"
        "  -h, --help           show this help and exit\n"
        "\n"
        "SWD clock calibration:\n"
        "      --speed-coeff N  delay-loop iterations per kHz    (default: %u)\n"
        "      --speed-offset N fixed per-transition cost, in the same iterations\n"
        "                       (default: %u)\n"
        "      --auto-calibrate re-measure speed-coeff/speed-offset on this machine\n"
        "                       at startup instead of using the two above; also\n"
        "                       available at runtime as the Calibrate vendor\n"
        "                       command (0x91), which reports the values it measured\n"
        "\n"
        "      --clock HZ       startup SWD clock, in Hz (default: %u kHz)\n"
        "\n"
        "The clock starts at --clock (%u kHz by default) and stays there only until\n"
        "the client picks one: OpenOCD sends its own during init, in response to\n"
        "\"adapter speed\".\n"
        "\n"
        "That \"adapter speed <khz>\" is turned into a delay of\n"
        "ceil(speed_coeff / khz) - speed_offset loop iterations per SWCLK half\n"
        "period, so the fastest reachable clock is speed_coeff / speed_offset kHz\n"
        "(%u kHz by default). These are the same two numbers as OpenOCD's\n"
        "\"bcm2835gpio speed_coeffs\", and the defaults are its measurements for a\n"
        "Pi 2 and later. Measure them for your own board if the clock OpenOCD asks\n"
        "for and the clock on a scope disagree, or pass --auto-calibrate to have\n"
        "this process measure its own delay loop and GPIO write cost instead --\n"
        "no scope needed, though it is still only as good as clock_gettime() and\n"
        "whatever else the CPU is doing at the time.\n"
        "\n"
        "Pin options (BCM numbering):\n"
        "      --swclk N        SWCLK                         (default: %u)\n"
        "      --swdio N        single bidirectional SWDIO pin, wired straight to the\n"
        "                       target: one pin carries both directions and the GPIO\n"
        "                       itself is flipped between input and output\n"
        "      --swdio-in N     SWDIO in,  split wiring       (default: %u)\n"
        "      --swdio-out N    SWDIO out, split wiring       (default: %u)\n"
        "      --swdio-dir N    buffer direction pin          (default: %u)\n"
        "\n"
        "SWDIO defaults to the split wiring above: separate input and output pins\n"
        "behind a buffer whose direction is driven by --swdio-dir. Pass --swdio\n"
        "instead for a single bidirectional pin; it cannot be combined with\n"
        "--swdio-in/--swdio-out, but may be combined with --swdio-dir when one data\n"
        "pin sits behind a buffer that still needs a direction line. With --swdio and\n"
        "no --swdio-dir, no direction pin is driven at all.\n"
        "\n"
        "The trailing positional arguments are the older interface, kept working for\n"
        "existing callers; they mean the same as -b, -p and -s respectively.\n",
        argv0, PORT_DEFAULT,
        I2C_DEV_DEFAULT, I2C_ADDR_DEFAULT,
        logging::level_name(LOG_LEVEL_DEFAULT),
        cmsis::dap::SPEED_COEFF_DEFAULT, cmsis::dap::SPEED_OFFSET_DEFAULT,
        cmsis::dap::CLOCK_KHZ_DEFAULT,
        cmsis::dap::CLOCK_KHZ_DEFAULT,
        cmsis::dap::SPEED_COEFF_DEFAULT / cmsis::dap::SPEED_OFFSET_DEFAULT,
        SWCLK_DEFAULT, SWDIO_IN_DEFAULT,
        SWDIO_OUT_DEFAULT, SWDIO_DIR_DEFAULT);
}

// Parse one integer option value, rejecting anything with trailing junk or out
// of [lo, hi]. `base` is passed through to strtol(), so callers that want to
// accept 0x-prefixed hex (--i2c-addr) pass 0 and the rest pass 10.
static bool parse_int_opt(const char *name, const char *arg,
                          long lo, long hi, int base, long *out)
{
    char *end = NULL;
    errno = 0;
    long v = strtol(arg, &end, base);
    if (errno != 0 || end == arg || *end != '\0' || v < lo || v > hi) {
        LOGE_KV("invalid option value",
                "option=%s value=\"%s\" min=%ld max=%ld", name, arg, lo, hi);
        return false;
    }
    *out = v;
    return true;
}

// Flags take precedence; any leftover positional arguments are then applied in
// the historical order (bind address, port, mux position) to whichever of those
// three no flag has already set.
static bool parse_args(int argc, char **argv, Options *opt)
{
    bool have_bind = false;
    bool have_port = false;
    bool have_swdio_split = false;  // --swdio-in / --swdio-out: split pair
    bool have_swdio_dir = false;
    long v = 0;
    int c;

    while ((c = getopt_long(argc, argv, "b:p:s:Ml:h", LONG_OPTS, NULL)) != -1) {
        switch (c) {
        case 'b':
            opt->bind_addr = optarg;
            have_bind = true;
            break;
        case 'p':
            if (!parse_int_opt("port", optarg, 1, 65535, 10, &v)) return false;
            opt->port = (int)v;
            have_port = true;
            break;
        case 's':
            if (!parse_int_opt("swd mux position", optarg, 0, 12, 10, &v)) return false;
            opt->swd_pos = (unsigned int)v;
            opt->has_swd_pos = true;
            break;
        case 'M':
            opt->mux_enabled = false;
            break;
        case OPT_SWCLK:
            if (!parse_int_opt("--swclk", optarg, 0, PIN_ARG_MAX, 10, &v)) return false;
            opt->swclk = (unsigned int)v;
            break;
        case OPT_SWDIO:
            if (!parse_int_opt("--swdio", optarg, 0, PIN_ARG_MAX, 10, &v)) return false;
            opt->swdio_out = (unsigned int)v;
            opt->swdio_single = true;
            break;
        case OPT_SWDIO_IN:
            if (!parse_int_opt("--swdio-in", optarg, 0, PIN_ARG_MAX, 10, &v)) return false;
            opt->swdio_in = (unsigned int)v;
            have_swdio_split = true;
            break;
        case OPT_SWDIO_OUT:
            if (!parse_int_opt("--swdio-out", optarg, 0, PIN_ARG_MAX, 10, &v)) return false;
            opt->swdio_out = (unsigned int)v;
            have_swdio_split = true;
            break;
        case OPT_SWDIO_DIR:
            if (!parse_int_opt("--swdio-dir", optarg, 0, PIN_ARG_MAX, 10, &v)) return false;
            opt->swdio_dir = (unsigned int)v;
            have_swdio_dir = true;
            break;
        case OPT_I2C_DEV:
            opt->i2c_dev = optarg;
            break;
        case OPT_I2C_ADDR:
            if (!parse_int_opt("--i2c-addr", optarg, 0x00, 0x7f, 0, &v)) return false;
            opt->i2c_addr = (unsigned int)v;
            break;
        case OPT_SPEED_COEFF:
            if (!parse_int_opt("--speed-coeff", optarg, 1, 100000000, 10, &v)) return false;
            opt->speed_coeff = (unsigned int)v;
            break;
        case OPT_SPEED_OFFSET:
            // At least 1: it is a divisor when reporting the clock ceiling, and
            // a zero fixed cost would claim an unbounded maximum clock.
            if (!parse_int_opt("--speed-offset", optarg, 1, 1000000, 10, &v)) return false;
            opt->speed_offset = (unsigned int)v;
            break;
        case OPT_AUTO_CALIBRATE:
            opt->auto_calibrate = true;
            break;
        case OPT_CLOCK:
            if (!parse_int_opt("--clock", optarg, 1, 1000000000, 10, &v)) return false;
            opt->clock_hz = (unsigned int)v;
            break;
        case OPT_UNIX_SOCKET:
            opt->unix_socket = optarg;
            break;
        case 'l':
            if (!logging::parse_level(optarg, &opt->log_level)) {
                LOGE_KV("invalid log level",
                        "value=%s expected=\"error warn info debug\"", optarg);
                return false;
            }
            break;
        case 'h':
            print_usage(argv[0]);
            exit(0);
        default:
            return false;   // getopt_long already printed the reason
        }
    }

    if (opt->speed_offset >= opt->speed_coeff) {
        // The ceiling is speed_coeff / speed_offset kHz, so an offset at or
        // above the coefficient means every requested clock lands on the
        // no-delay path and "adapter speed" stops meaning anything at all.
        LOGE_KV("--speed-offset must be below --speed-coeff",
                "speed_coeff=%u speed_offset=%u",
                opt->speed_coeff, opt->speed_offset);
        return false;
    }

    if (opt->swdio_single && have_swdio_split) {
        LOGE("--swdio is a single bidirectional pin; it cannot be combined "
                  "with --swdio-in/--swdio-out");
        return false;
    }
    // Only drive a direction pin if the board actually has one; a bare
    // bidirectional pin has nothing to point anywhere.
    if (opt->swdio_single) {
        opt->swdio_has_dir = have_swdio_dir;
    }

    // Legacy positional interface.
    int npos = argc - optind;
    if (npos > 3) {
        LOGE("too many arguments (expected at most bind address, port, mux position)");
        return false;
    }
    if (npos > 0 && argv[optind][0] != '\0' && !have_bind) {
        opt->bind_addr = argv[optind];
    }
    if (npos > 1 && !have_port) {
        if (!parse_int_opt("port", argv[optind + 1], 1, 65535, 10, &v)) return false;
        opt->port = (int)v;
    }
    if (npos > 2 && !opt->has_swd_pos) {
        if (!parse_int_opt("swd mux position", argv[optind + 2], 0, 12, 10, &v)) return false;
        opt->swd_pos = (unsigned int)v;
        opt->has_swd_pos = true;
    }

    if (opt->has_swd_pos && !opt->mux_enabled) {
        LOGE("--swd-pos and --no-swd-mux are mutually exclusive");
        return false;
    }

    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;

    // Under systemd/journald or any pipe, stdout is block-buffered by default,
    // which reorders the info stream against unbuffered stderr and drops
    // whatever is still pending if the process dies. Line buffering keeps the
    // log readable and complete.
    setvbuf(stdout, NULL, _IOLBF, 0);

    if (!parse_args(argc, argv, &opt)) {
        print_usage(argv[0]);
        return 1;
    }

    logging::init(opt.log_level);

    bool pins_ok = opt.swdio_single
        ? gpio::configure_single(opt.swclk, opt.swdio_out, opt.swdio_has_dir, opt.swdio_dir)
        : gpio::configure_split(opt.swclk, opt.swdio_in, opt.swdio_out, opt.swdio_dir);
    if (!pins_ok) {
        // A bad pin mapping is CLI misuse like any other, so show the flags.
        print_usage(argv[0]);
        return 1;
    }

    if (opt.swdio_single) {
        LOGI_KV("swdio wiring", "mode=single swclk=%u swdio=%u has_dir=%s",
                opt.swclk, opt.swdio_out, opt.swdio_has_dir ? "true" : "false");
        if (opt.swdio_has_dir) {
            LOGI_KV("swdio direction pin", "swdio_dir=%u", opt.swdio_dir);
        }
    } else {
        LOGI_KV("swdio wiring",
                "mode=split swclk=%u swdio_in=%u swdio_out=%u swdio_dir=%u",
                opt.swclk, opt.swdio_in, opt.swdio_out, opt.swdio_dir);
    }

    swdmux::init(opt.i2c_dev, opt.i2c_addr, opt.mux_enabled);
    if (opt.mux_enabled) {
        LOGI_KV("swd mux enabled", "i2c_dev=%s i2c_addr=0x%02x",
                opt.i2c_dev, opt.i2c_addr);
    } else {
        LOGI("swd mux disabled");
    }

    if (opt.has_swd_pos) {
        if (!swdmux::select_pos(opt.swd_pos)) {
            return 1;
        }
    }

    if (opt.auto_calibrate) {
        LOGI("swd clock: auto-calibrating (--auto-calibrate given, ignoring "
             "--speed-coeff/--speed-offset defaults for now)");
    } else {
        LOGI_KV("swd clock",
                "initial_hz=%u speed_coeff=%u speed_offset=%u max_khz=%u",
                opt.clock_hz, opt.speed_coeff, opt.speed_offset,
                opt.speed_coeff / opt.speed_offset);
    }

    tcp_server::install_exit_handlers();

    // init() maps the gpio registers (via DAP_SETUP()), which calibrate()
    // needs done first -- see cmsis::dap::calibrate()'s contract in DAP.h.
    cmsis::dap::init(opt.speed_coeff, opt.speed_offset, opt.clock_hz);

    if (opt.auto_calibrate) {
        unsigned int speed_coeff = 0;
        unsigned int speed_offset = 0;
        if (cmsis::dap::calibrate(&speed_coeff, &speed_offset)) {
            LOGI_KV("swd clock calibrated",
                    "initial_hz=%u speed_coeff=%u speed_offset=%u max_khz=%u",
                    opt.clock_hz, speed_coeff, speed_offset,
                    speed_coeff / speed_offset);
        } else {
            LOGW_KV("swd clock calibration failed, keeping flag/default values",
                    "speed_coeff=%u speed_offset=%u",
                    opt.speed_coeff, opt.speed_offset);
        }
    }

    tcp_server::serve(opt.bind_addr, opt.port, opt.unix_socket);
    cmsis::dap::shutdown();

    LOGI("exiting");
    return 0;
}
