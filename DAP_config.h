/*
 * Copyright (c) 2013-2022 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------
 *
 * Project:      CMSIS-DAP Source
 * Title:        DAP_config.h CMSIS-DAP Configuration File
 *
 * NOTE:
 * This is the CMSIS-DAP DAP_config.h port for this wrapper. Upstream assumes
 * the code runs on an ARM MCU; here it runs on Linux on a Raspberry Pi, so the
 * comments below referring to Cortex-M, cycle-accurate timing, etc should be
 * disregarded. The I/O pin abstraction is implemented on top of the mmap'ed
 * GPIO registers in gpio_regs.h, and timing is best-effort only.
 *
 * Follows main/DAP_config.h of
 * https://github.com/bkuschak/cmsis_dap_tcp_esp32/, which does the same for
 * the ESP32 GPIO. As noted there, the clock calibration has to be measured on
 * the hardware in question; this port keeps it in DAP.h rather than here,
 * because it is expressed the way OpenOCD's own Pi bitbang driver expresses
 * it and not the way upstream CMSIS-DAP does.
 */

#ifndef DAP_CONFIG_H
#define DAP_CONFIG_H

#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "gpio.h"
#include "gpio_regs.h"

/**************************************************************************************************
\defgroup DAP_Config_Debug_gr CMSIS-DAP Debug Unit Information
\ingroup DAP_ConfigIO_gr
@{
Provides definitions about the hardware and configuration of the Debug Unit.

This information includes:
 - Definition of Cortex-M processor parameters used in CMSIS-DAP Debug Unit.
 - Debug Unit Identification strings (Vendor, Product, Serial Number).
 - Debug Unit communication packet size.
 - Debug Access Port supported modes and settings (JTAG/SWD and SWO).
 - Optional information about a connected Target Device (for Evaluation Boards).
*/

// Don't use ARM assembler.
#define __CC_ARM

// Upstream calibrates the bitbang clock here, with CPU_CLOCK,
// IO_PORT_WRITE_CYCLES and DELAY_SLOW_CYCLES. None of the three is measurable
// from Linux userspace, so this port calibrates in OpenOCD's terms instead --
// see SPEED_COEFF_DEFAULT in DAP.h and Set_Clock_Delay() in DAP.cpp.

/// Indicate that Serial Wire Debug (SWD) communication mode is available at the Debug Access Port.
#define DAP_SWD                 1

/// Indicate that JTAG communication mode is available at the Debug Port.
#define DAP_JTAG                0

/// Configure maximum number of JTAG devices on the scan chain connected to the Debug Access Port.
#define DAP_JTAG_DEV_CNT        8U

/// Default communication mode on the Debug Access Port.
#define DAP_DEFAULT_PORT        1U

// Upstream's default SWD/JTAG clock lives here as DAP_DEFAULT_SWJ_CLOCK. This
// port takes it from the command line instead -- see CLOCK_KHZ_DEFAULT in
// DAP.h and --speed.

/// Maximum Package Size for Command and Response data.
#define DAP_PACKET_SIZE         1024U

/// Maximum Package Buffers for Command and Response data.
#define DAP_PACKET_COUNT        8U

/// Indicate that UART Serial Wire Output (SWO) trace is available.
#define SWO_UART                0

/// Indicate that Manchester Serial Wire Output (SWO) trace is available.
#define SWO_MANCHESTER          0

/// SWO Trace Buffer Size.
#define SWO_BUFFER_SIZE         4096U

/// SWO Streaming Trace.
#define SWO_STREAM              0

/// Clock frequency of the Test Domain Timer.
#define TIMESTAMP_CLOCK         1000000000U

/// Indicate that UART Communication Port is available.
#define DAP_UART                0

/// Indicate that UART Communication via USB COM Port is available.
#define DAP_UART_USB_COM_PORT   0

/// Debug Unit is connected to fixed Target Device.
#define TARGET_DEVICE_VENDOR    "Arm"
#define TARGET_DEVICE_NAME      "Cortex-M"
#define TARGET_BOARD_VENDOR     "Raspberry Pi"
#define TARGET_BOARD_NAME       "Raspberry Pi 4"

static const char TargetDeviceVendor [] = TARGET_DEVICE_VENDOR;
static const char TargetDeviceName   [] = TARGET_DEVICE_NAME;
static const char TargetBoardVendor  [] = TARGET_BOARD_VENDOR;
static const char TargetBoardName    [] = TARGET_BOARD_NAME;

/** Get Vendor Name string.
\param str Pointer to buffer to store the string (max 60 characters).
\return String length (including terminating NULL character) or 0 (no string).
*/
static inline uint8_t DAP_GetVendorString (char *str)
{
    const char *vendor = "OpenOCD";
    int maxlen = 60;
    strncpy(str, vendor, maxlen);
    str[maxlen-1] = '\0';
    return strlen(str) + 1;
}

/** Get Product Name string.
\param str Pointer to buffer to store the string (max 60 characters).
\return String length (including terminating NULL character) or 0 (no string).
*/
static inline uint8_t DAP_GetProductString (char *str)
{
    const char *product = "Raspberry Pi 4 CMSIS-DAP device";
    int maxlen = 60;
    strncpy(str, product, maxlen);
    str[maxlen-1] = '\0';
    return strlen(str) + 1;
}

/** Get Serial Number string.
\param str Pointer to buffer to store the string (max 60 characters).
\return String length (including terminating NULL character) or 0 (no string).
*/
static inline uint8_t DAP_GetSerNumString (char *str)
{
    int maxlen = 60;
    snprintf(str, maxlen, "RP4-%d", getpid());
    return strlen(str) + 1;
}

/** Get Target Device Vendor string.
\param str Pointer to buffer to store the string (max 60 characters).
\return String length (including terminating NULL character) or 0 (no string).
*/
static inline uint8_t DAP_GetTargetDeviceVendorString (char *str)
{
    uint8_t len;

    strcpy(str, TargetDeviceVendor);
    len = (uint8_t)(strlen(TargetDeviceVendor) + 1U);
    return (len);
}

/** Get Target Device Name string.
\param str Pointer to buffer to store the string (max 60 characters).
\return String length (including terminating NULL character) or 0 (no string).
*/
static inline uint8_t DAP_GetTargetDeviceNameString (char *str)
{
    uint8_t len;

    strcpy(str, TargetDeviceName);
    len = (uint8_t)(strlen(TargetDeviceName) + 1U);
    return (len);
}

/** Get Target Board Vendor string.
\param str Pointer to buffer to store the string (max 60 characters).
\return String length (including terminating NULL character) or 0 (no string).
*/
static inline uint8_t DAP_GetTargetBoardVendorString (char *str)
{
    uint8_t len;

    strcpy(str, TargetBoardVendor);
    len = (uint8_t)(strlen(TargetBoardVendor) + 1U);
    return (len);
}

/** Get Target Board Name string.
\param str Pointer to buffer to store the string (max 60 characters).
\return String length (including terminating NULL character) or 0 (no string).
*/
static inline uint8_t DAP_GetTargetBoardNameString (char *str)
{
    uint8_t len;

    strcpy(str, TargetBoardName);
    len = (uint8_t)(strlen(TargetBoardName) + 1U);
    return (len);
}

/** Get Product Firmware Version string.
\param str Pointer to buffer to store the string (max 60 characters).
\return String length (including terminating NULL character) or 0 (no string).
*/
static inline uint8_t DAP_GetProductFirmwareVersionString (char *str)
{
    (void)str;
    return (0U);
}

///@}


//**************************************************************************************************
/**
\defgroup DAP_Config_PortIO_gr CMSIS-DAP Hardware I/O Pin Access
\ingroup DAP_ConfigIO_gr
@{

Standard I/O Pins of the CMSIS-DAP Hardware Debug Port support standard JTAG mode
and Serial Wire Debug (SWD) mode. In SWD mode only 2 pins are required to implement the debug
interface of a device. The following I/O Pins are provided:

JTAG I/O Pin                 | SWD I/O Pin          | CMSIS-DAP Hardware pin mode
---------------------------- | -------------------- | ---------------------------------------------
TCK: Test Clock              | SWCLK: Clock         | Output Push/Pull
TMS: Test Mode Select        | SWDIO: Data I/O      | Output Push/Pull; Input (for receiving data)
TDI: Test Data Input         |                      | Output Push/Pull
TDO: Test Data Output        |                      | Input
nTRST: Test Reset (optional) |                      | Output Open Drain with pull-up resistor
nRESET: Device Reset         | nRESET: Device Reset | Output Open Drain with pull-up resistor
*/


// Configure DAP I/O pins ------------------------------

/** Setup SWD I/O pins: SWCLK, SWDIO, and nRESET.
Configures the DAP Hardware I/O pins for Serial Wire Debug (SWD) mode:
 - SWCLK, SWDIO, nRESET to output mode and set to default high level.
 - TDI, nTRST to HighZ mode (pins are unused in SWD mode).
*/
static inline void PORT_SWD_SETUP (void)
{
    gpio::init_once();
    gpio::drive_output(gpio::swclk, 0);
    if (!gpio::swdio_single_pin) {
        gpio::set_input(gpio::swdio_in);
    }
    if (gpio::swdio_has_dir) {
        gpio::drive_output(gpio::swdio_dir, 0);   // 0 = host drives target
    }
    // Last, so that on a single bidirectional pin the buffer (if any) is
    // already pointed at the target before this pin starts driving.
    gpio::drive_output(gpio::swdio_out, 0);
    gpio::swdio_is_output = 1;
}

/** Disable JTAG/SWD I/O Pins.
Disables the DAP Hardware I/O pins which configures:
 - TCK/SWCLK, TMS/SWDIO, TDI, TDO, nTRST, nRESET to High-Z mode.
*/
static inline void PORT_OFF (void)
{
    gpio::set_input(gpio::swclk);
    gpio::set_input(gpio::swdio_in);
    gpio::set_input(gpio::swdio_out);     // the same pin as SWDIO_IN when single
    if (gpio::swdio_has_dir) {
        // Keep driving the direction pin, pointed away from the target. Left
        // as an input it floats, and a board that pulls it to the "host
        // drives target" level would have the buffer driving the target's
        // SWDIO from a data pin that is now Hi-Z -- i.e. with garbage. High is
        // the only safe resting state: the buffer faces the host and the
        // target's SWDIO is left alone.
        gpio::drive_output(gpio::swdio_dir, 1);
    }
    gpio::swdio_is_output = 0;
}

// SWCLK/TCK I/O pin -------------------------------------

/** SWCLK/TCK I/O pin: Get Input.
\return Current status of the SWCLK/TCK DAP hardware I/O pin.
*/
static inline uint32_t PIN_SWCLK_TCK_IN  (void)
{
    return gpio::read_level(gpio::swclk);
}

static inline void     PIN_SWCLK_TCK_SET (void)
{
    gpio::set_high(gpio::swclk);
}

static inline void     PIN_SWCLK_TCK_CLR (void)
{
    gpio::set_low(gpio::swclk);
}

static inline uint32_t PIN_SWDIO_TMS_IN  (void)
{
    return gpio::read_level(gpio::swdio_in);
}

static inline void     PIN_SWDIO_TMS_SET (void)
{
    gpio::set_high(gpio::swdio_out);
}

static inline void     PIN_SWDIO_TMS_CLR (void)
{
    gpio::set_low(gpio::swdio_out);
}

static inline uint32_t PIN_SWDIO_IN      (void)
{
    return gpio::read_level(gpio::swdio_in);
}

static inline void     PIN_SWDIO_OUT     (uint32_t bit)
{
    if (bit & 1U) {
        gpio::set_high(gpio::swdio_out);
    } else {
        gpio::set_low(gpio::swdio_out);
    }
}

/** SWDIO I/O pin: Switch to Output mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to output mode. This function is
called prior \ref PIN_SWDIO_OUT function calls.
*/
static inline void     PIN_SWDIO_OUT_ENABLE  (void)
{
    if (!gpio::swdio_is_output) {
        gpio::request_swdio_output();
    }
}

/** SWDIO I/O pin: Switch to Input mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to input mode. This function is
called prior \ref PIN_SWDIO_IN function calls.
*/
static inline void     PIN_SWDIO_OUT_DISABLE (void)
{
    if (gpio::swdio_is_output) {
        gpio::request_swdio_input();
    }
}


// TDI Pin I/O ---------------------------------------------

/** TDI I/O pin: Get Input.
\return Current status of the TDI DAP hardware I/O pin.
*/
static inline uint32_t PIN_TDI_IN  (void)
{
    return 0;
}

/** TDI I/O pin: Set Output.
\param bit Output value for the TDI DAP hardware I/O pin.
*/
static inline void     PIN_TDI_OUT (uint32_t bit)
{
    (void)bit;
}


// TDO Pin I/O ---------------------------------------------

/** TDO I/O pin: Get Input.
\return Current status of the TDO DAP hardware I/O pin.
*/
static inline uint32_t PIN_TDO_IN  (void)
{
    return 0;
}


// nTRST Pin I/O -------------------------------------------

/** nTRST I/O pin: Get Input.
\return Current status of the nTRST DAP hardware I/O pin.
*/
static inline uint32_t PIN_nTRST_IN   (void)
{
    return 0;
}

/** nTRST I/O pin: Set Output.
\param bit JTAG TRST Test Reset pin status:
           - 0: issue a JTAG TRST Test Reset.
           - 1: release JTAG TRST Test Reset.
*/
static inline void     PIN_nTRST_OUT  (uint32_t bit)
{
    (void)bit;
}

// nRESET Pin I/O------------------------------------------

/** nRESET I/O pin: Get Input.
\return Current status of the nRESET DAP hardware I/O pin.
*/
static inline uint32_t PIN_nRESET_IN  (void)
{
    return 0;
}

/** nRESET I/O pin: Set Output.
\param bit target device hardware reset pin status:
           - 0: issue a device hardware reset.
           - 1: release device hardware reset.
*/
static inline void     PIN_nRESET_OUT (uint32_t bit)
{
    (void)bit;
}

///@}


//**************************************************************************************************
/**
\defgroup DAP_Config_Timestamp_gr CMSIS-DAP Timestamp
\ingroup DAP_ConfigIO_gr
@{
Access function for Test Domain Timer.

The value of the Test Domain Timer in the Debug Unit is returned by the function \ref TIMESTAMP_GET. By
default, the DWT timer is used.  The frequency of this timer is configured with \ref TIMESTAMP_CLOCK.

*/

/** Get timestamp of Test Domain Timer.
\return Current timestamp value.
*/
static inline uint32_t TIMESTAMP_GET (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint32_t)(((uint64_t)ts.tv_sec * 1000000000ULL) + (uint64_t)ts.tv_nsec);
}

///@}


//**************************************************************************************************
/**
\defgroup DAP_Config_Initialization_gr CMSIS-DAP Initialization
\ingroup DAP_ConfigIO_gr
@{

CMSIS-DAP Hardware I/O and LED Pins are initialized with the function \ref DAP_SETUP.
*/

/** Setup of the Debug Unit I/O pins and LEDs (called when Debug Unit is initialized).
This function performs the initialization of the CMSIS-DAP Hardware I/O Pins and the
Status LEDs. In detail the operation of Hardware I/O and LED pins are enabled and set:
 - I/O clock system enabled.
 - all I/O pins: input buffer enabled, output pins are set to HighZ mode.
 - for nTRST, nRESET a weak pull-up (if available) is enabled.
 - LED output pins are enabled and LEDs are turned off.
*/
static inline void DAP_SETUP (void)
{
    gpio::init_once();
    PORT_OFF();
}

/** Reset Target Device with custom specific I/O pin or command sequence.
This function allows the optional implementation of a device specific reset sequence.
It is called when the command \ref DAP_ResetTarget and is for example required
when a device needs a time-critical unlock sequence that enables the debug port.
\return 0 = no device specific reset sequence is implemented.\n
        1 = a device specific reset sequence is implemented.
*/
static inline uint8_t RESET_TARGET (void)
{
    return (0U);             // change to '1' when a device reset sequence is implemented
}

///@}

#endif  // DAP_CONFIG_H
