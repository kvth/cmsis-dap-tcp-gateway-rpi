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
 * $Date:        5. December 2022
 * $Revision:    V2.1.2
 *
 * Project:      CMSIS-DAP Source
 * Title:        DAP.c CMSIS-DAP Commands
 *
 * NOTE:
 * This came from CMSIS-DAP
 * (https://github.com/ARMmbed/DAPLink/blob/main/source/daplink/cmsis-dap/DAP.c),
 * which assumes the code is running on an ARM MCU. Here it runs on Linux on a
 * Raspberry Pi and bitbangs SWD over the GPIO registers (see gpio.h), so
 * comments referring to Cortex-M, cycle-accurate timing, etc should be
 * disregarded.
 *
 * The same retargeting was done for the ESP32 in
 * https://github.com/bkuschak/cmsis_dap_tcp_esp32/ (main/DAP.c, main/SW_DP.c),
 * which this port follows. SW_DP.c's bitbang is merged in here rather than
 * kept in its own file, and DAP_Data is a plain global instead of a
 * thread-local, since one process serves one interface.
 */

#include "DAP.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "DAP_protocol.h"
#include "gpio.h"
#include "calibrate.h"
#include "dp_connect.h"
#include "logging.h"
#include "node_query.h"
#include "rp2040.h"
#include "rtt.h"
#include "swd_port.h"
#include "swdmux.h"

static_assert(cmsis::dap::MAX_PACKET_SIZE == DAP_PACKET_SIZE,
              "the packet size callers allocate for must match the one the "
              "command handlers were built with");
static_assert(cmsis::dap::MAX_PACKET_COUNT == DAP_PACKET_COUNT,
              "the packet count callers size their buffers from must match the "
              "one reported to the client by DAP_Info");
static_assert(swd::REQ_APnDP == DAP_TRANSFER_APnDP &&
              swd::REQ_RnW == DAP_TRANSFER_RnW &&
              swd::REQ_ADDR == (DAP_TRANSFER_A2 | DAP_TRANSFER_A3) &&
              swd::ACK_OK == DAP_TRANSFER_OK &&
              swd::SEQ_CAPTURE == SWD_SEQUENCE_DIN,
              "swd_port.h repeats the transfer encoding for callers outside "
              "this file; the two must agree");

DAP_Data_t DAP_Data;                    // DAP Data
volatile uint8_t DAP_TransferAbort;     // Transfer Abort Flag

// Configurable delay for clock generation.
//
// A plain register counter around an empty inline-asm statement, the same
// barrier idiom PIN_DELAY_FAST() below already uses: the memory clobber is
// what stops the compiler proving the loop has no effect and deleting it,
// and keeping the counter itself in a register (not behind `volatile`) is
// what keeps one iteration's cost small and consistent -- which matters
// because speed_coeff/speed_offset (see DAP.h) are only meaningful relative
// to whatever this loop actually costs per iteration on the machine they
// were measured on.
static inline void PIN_DELAY_SLOW (uint32_t delay) {
  for (uint32_t i = 0; i < delay; i++) {
    __asm__ __volatile__("" ::: "memory");
  }
}

// Fixed delay for fast clock generation
#ifndef DELAY_FAST_CYCLES
#define DELAY_FAST_CYCLES       0U      // Number of cycles: 0..3
#endif
static inline void PIN_DELAY_FAST (void) {
#if (DELAY_FAST_CYCLES >= 1U)
  __asm__ __volatile__("" ::: "memory");
#endif
#if (DELAY_FAST_CYCLES >= 2U)
  __asm__ __volatile__("" ::: "memory");
#endif
#if (DELAY_FAST_CYCLES >= 3U)
  __asm__ __volatile__("" ::: "memory");
#endif
}

// SW Macros

#define PIN_SWCLK_SET PIN_SWCLK_TCK_SET
#define PIN_SWCLK_CLR PIN_SWCLK_TCK_CLR

#define SW_CLOCK_CYCLE()                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_WRITE_BIT(bit)               \
  PIN_SWDIO_OUT(bit);                   \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_READ_BIT(bit)                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  bit = PIN_SWDIO_IN();                 \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)


// Generate SWJ Sequence
//   count:  sequence bit count
//   data:   pointer to sequence bit data
//   return: none
static void SWJ_Sequence (uint32_t count, const uint8_t *data) {
  uint32_t val;
  uint32_t n;

  val = 0U;
  n = 0U;
  while (count--) {
    if (n == 0U) {
      val = *data++;
      n = 8U;
    }
    if (val & 1U) {
      PIN_SWDIO_TMS_SET();
    } else {
      PIN_SWDIO_TMS_CLR();
    }
    SW_CLOCK_CYCLE();
    val >>= 1;
    n--;
  }
}

// Generate SWD Sequence
//   info:   sequence information
//   swdo:   pointer to SWDIO generated data
//   swdi:   pointer to SWDIO captured data
//   return: none
static void SWD_Sequence (uint32_t info, const uint8_t *swdo, uint8_t *swdi) {
  uint32_t val;
  uint32_t bit;
  uint32_t n, k;

  n = info & SWD_SEQUENCE_CLK;
  if (n == 0U) {
    n = 64U;
  }

  if (info & SWD_SEQUENCE_DIN) {
    while (n) {
      val = 0U;
      for (k = 8U; k && n; k--, n--) {
        SW_READ_BIT(bit);
        val >>= 1;
        val  |= bit << 7;
      }
      val >>= k;
      *swdi++ = (uint8_t)val;
    }
  } else {
    while (n) {
      val = *swdo++;
      for (k = 8U; k && n; k--, n--) {
        SW_WRITE_BIT(val);
        val >>= 1;
      }
    }
  }
}

// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
#define SWD_TransferFunction(speed)     /**/                                    \
static uint8_t SWD_Transfer##speed (uint32_t request, uint32_t *data) {         \
  uint32_t ack;                                                                 \
  uint32_t bit;                                                                 \
  uint32_t val;                                                                 \
  uint32_t parity;                                                              \
                                                                                \
  uint32_t n;                                                                   \
                                                                                \
  /* Packet Request */                                                          \
  parity = 0U;                                                                  \
  SW_WRITE_BIT(1U);                     /* Start Bit */                         \
  bit = request >> 0;                                                           \
  SW_WRITE_BIT(bit);                    /* APnDP Bit */                         \
  parity += bit;                                                                \
  bit = request >> 1;                                                           \
  SW_WRITE_BIT(bit);                    /* RnW Bit */                           \
  parity += bit;                                                                \
  bit = request >> 2;                                                           \
  SW_WRITE_BIT(bit);                    /* A2 Bit */                            \
  parity += bit;                                                                \
  bit = request >> 3;                                                           \
  SW_WRITE_BIT(bit);                    /* A3 Bit */                            \
  parity += bit;                                                                \
  SW_WRITE_BIT(parity);                 /* Parity Bit */                        \
  SW_WRITE_BIT(0U);                     /* Stop Bit */                          \
  SW_WRITE_BIT(1U);                     /* Park Bit */                          \
                                                                                \
  /* Turnaround */                                                              \
  PIN_SWDIO_OUT_DISABLE();                                                      \
  for (n = DAP_Data.swd_conf.turnaround; n; n--) {                              \
    SW_CLOCK_CYCLE();                                                           \
  }                                                                             \
                                                                                \
  /* Acknowledge response */                                                    \
  SW_READ_BIT(bit);                                                             \
  ack  = bit << 0;                                                              \
  SW_READ_BIT(bit);                                                             \
  ack |= bit << 1;                                                              \
  SW_READ_BIT(bit);                                                             \
  ack |= bit << 2;                                                              \
                                                                                \
  if (ack == DAP_TRANSFER_OK) {         /* OK response */                       \
    /* Data transfer */                                                         \
    if (request & DAP_TRANSFER_RnW) {                                           \
      /* Read data */                                                           \
      val = 0U;                                                                 \
      parity = 0U;                                                              \
      for (n = 32U; n; n--) {                                                   \
        SW_READ_BIT(bit);               /* Read RDATA[0:31] */                  \
        parity += bit;                                                          \
        val >>= 1;                                                              \
        val  |= bit << 31;                                                      \
      }                                                                         \
      SW_READ_BIT(bit);                 /* Read Parity */                       \
      if ((parity ^ bit) & 1U) {                                                \
        ack = DAP_TRANSFER_ERROR;                                               \
      }                                                                         \
      if (data) { *data = val; }                                                \
      /* Turnaround */                                                          \
      for (n = DAP_Data.swd_conf.turnaround; n; n--) {                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
      PIN_SWDIO_OUT_ENABLE();                                                   \
    } else {                                                                    \
      /* Turnaround */                                                          \
      for (n = DAP_Data.swd_conf.turnaround; n; n--) {                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
      PIN_SWDIO_OUT_ENABLE();                                                   \
      /* Write data */                                                          \
      val = *data;                                                              \
      parity = 0U;                                                              \
      for (n = 32U; n; n--) {                                                   \
        SW_WRITE_BIT(val);              /* Write WDATA[0:31] */                 \
        parity += val;                                                          \
        val >>= 1;                                                              \
      }                                                                         \
      SW_WRITE_BIT(parity);             /* Write Parity Bit */                  \
    }                                                                           \
    /* Capture Timestamp */                                                     \
    if (request & DAP_TRANSFER_TIMESTAMP) {                                     \
      DAP_Data.timestamp = TIMESTAMP_GET();                                     \
    }                                                                           \
    /* Idle cycles */                                                           \
    n = DAP_Data.transfer.idle_cycles;                                          \
    if (n) {                                                                    \
      PIN_SWDIO_OUT(0U);                                                        \
      for (; n; n--) {                                                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
    }                                                                           \
    PIN_SWDIO_OUT(1U);                                                          \
    return ((uint8_t)ack);                                                      \
  }                                                                             \
                                                                                \
  if ((ack == DAP_TRANSFER_WAIT) || (ack == DAP_TRANSFER_FAULT)) {              \
    /* WAIT or FAULT response */                                                \
    if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) != 0U)) { \
      for (n = 32U+1U; n; n--) {                                                \
        SW_CLOCK_CYCLE();               /* Dummy Read RDATA[0:31] + Parity */   \
      }                                                                         \
    }                                                                           \
    /* Turnaround */                                                            \
    for (n = DAP_Data.swd_conf.turnaround; n; n--) {                            \
      SW_CLOCK_CYCLE();                                                         \
    }                                                                           \
    PIN_SWDIO_OUT_ENABLE();                                                     \
    if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) == 0U)) { \
      PIN_SWDIO_OUT(0U);                                                        \
      for (n = 32U+1U; n; n--) {                                                \
        SW_CLOCK_CYCLE();               /* Dummy Write WDATA[0:31] + Parity */  \
      }                                                                         \
    }                                                                           \
    PIN_SWDIO_OUT(1U);                                                          \
    return ((uint8_t)ack);                                                      \
  }                                                                             \
                                                                                \
  /* Protocol error */                                                          \
  for (n = DAP_Data.swd_conf.turnaround + 32U + 1U; n; n--) {                   \
    SW_CLOCK_CYCLE();                   /* Back off data phase */               \
  }                                                                             \
  PIN_SWDIO_OUT_ENABLE();                                                       \
  PIN_SWDIO_OUT(1U);                                                            \
  return ((uint8_t)ack);                                                        \
}


#undef  PIN_DELAY
#define PIN_DELAY() PIN_DELAY_FAST()
SWD_TransferFunction(Fast)

#undef  PIN_DELAY
#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)
SWD_TransferFunction(Slow)


// Shadow of the DP/AP registers the client caches, kept up to date by
// Shadow_Track() below so a probe-side burst can restore them. See swd_port.h
// for why this exists.
static swd::Shadow Shadow_Regs;
static bool        Shadow_Snoop = true;

// Fold one successful transfer into the shadow. Only the client's transfers
// count: a probe-side burst suspends tracking so its own writes, which it is
// about to undo, are not mistaken for the client's.
static void Shadow_Track(uint32_t request, const uint32_t *data) {
  if (!Shadow_Snoop) {
    return;
  }

  const uint32_t addr    = request & (DAP_TRANSFER_A2 | DAP_TRANSFER_A3);
  const bool     is_ap   = (request & DAP_TRANSFER_APnDP) != 0U;
  const bool     is_read = (request & DAP_TRANSFER_RnW) != 0U;

  if (!is_ap) {
    if (!is_read && (addr == swd::DP_REG_SELECT) && (data != NULL)) {
      Shadow_Regs.select       = *data;
      Shadow_Regs.select_valid = true;
    }
    return;
  }

  // Which AP register an AP transfer names depends on SELECT, so without a
  // SELECT value there is nothing we can attribute the access to -- and a
  // client that has not written SELECT has no cache of its own to protect.
  if (!Shadow_Regs.select_valid) {
    return;
  }
  if (((Shadow_Regs.select >> 4) & 0xFU) != 0U) {
    return;                             // not AP bank 0: not CSW/TAR/DRW
  }
  const uint8_t ap = (uint8_t)(Shadow_Regs.select >> 24);

  switch (addr) {
    case swd::AP_REG_CSW:
      if (!is_read && (data != NULL)) {
        Shadow_Regs.csw       = *data;
        Shadow_Regs.csw_ap    = ap;
        Shadow_Regs.csw_valid = true;
      }
      break;

    case swd::AP_REG_TAR:
      if (!is_read && (data != NULL)) {
        Shadow_Regs.tar       = *data;
        Shadow_Regs.tar_ap    = ap;
        Shadow_Regs.tar_valid = true;
      }
      break;

    case swd::AP_REG_DRW: {
      // The AP advances TAR itself on every DRW access when auto-increment is
      // on, so a client caching TAR has to model that; mirror the model, or
      // the value we restore is one the client stopped believing several
      // words ago. The increment wraps inside the 1kB auto-increment region
      // rather than carrying out of it.
      if (!Shadow_Regs.tar_valid || (Shadow_Regs.tar_ap != ap) ||
          !Shadow_Regs.csw_valid || (Shadow_Regs.csw_ap != ap)) {
        break;
      }
      uint32_t inc = 0U;
      switch ((Shadow_Regs.csw & swd::CSW_ADDRINC_MASK) >> 4) {
        // single
        case 1U:  inc = 1U << (Shadow_Regs.csw & swd::CSW_SIZE_MASK); break;
        case 2U:  inc = 4U; break;      // packed
        default:  break;                // off
      }
      if (inc != 0U) {
        const uint32_t region = swd::TAR_AUTOINC_REGION;
        Shadow_Regs.tar = (Shadow_Regs.tar & ~(region - 1U)) |
                          ((Shadow_Regs.tar + inc) & (region - 1U));
      }
      break;
    }

    default:
      break;
  }
}

// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
static uint8_t  SWD_Transfer(uint32_t request, uint32_t *data) {
  uint8_t ack;

  if (DAP_Data.fast_clock) {
    ack = SWD_TransferFast(request, data);
  } else {
    ack = SWD_TransferSlow(request, data);
  }

  if (ack == DAP_TRANSFER_OK) {
    Shadow_Track(request, data);
  }
  return ack;
}


// The swd_port.h interface. It sits here rather than in a file of its own
// because SWD_Transfer() and the shadow are what it exposes.
namespace swd {

void sequence_swj(uint32_t bits, const uint8_t *data) {
  SWJ_Sequence(bits, data);
}

void sequence_swd(uint8_t info, const uint8_t *out, uint8_t *in) {
  if ((info & SWD_SEQUENCE_DIN) != 0U) {
    PIN_SWDIO_OUT_DISABLE();
  } else {
    PIN_SWDIO_OUT_ENABLE();
  }
  SWD_Sequence(info, out, in);
  PIN_SWDIO_OUT_ENABLE();
}

uint8_t transfer(uint32_t request, uint32_t *data) {
  uint32_t retry = DAP_Data.transfer.retry_count;
  uint8_t  ack;

  do {
    ack = SWD_Transfer(request, data);
  } while ((ack == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);

  return ack;
}

const Shadow &shadow()  { return ::Shadow_Regs; }
void snoop_suspend()    { Shadow_Snoop = false; }
void snoop_resume()     { Shadow_Snoop = true; }

void shadow_reset() {
  ::Shadow_Regs = Shadow{};
  Shadow_Snoop = true;
}

bool port_is_swd() { return DAP_Data.debug_port == DAP_PORT_SWD; }

}  // namespace swd

// Clock calibration, settable at startup by init(). See DAP.h for what the two
// numbers mean and where the defaults come from.
static uint32_t Speed_Coeff  = cmsis::dap::SPEED_COEFF_DEFAULT;
static uint32_t Speed_Offset = cmsis::dap::SPEED_OFFSET_DEFAULT;

// The clock last requested through init() or DAP_SWJ_Clock, in Hertz. Kept
// so calibrate() can redo Set_Clock_Delay() against whatever was already in
// effect after it re-measures Speed_Coeff/Speed_Offset.
static uint32_t Current_Clock_Hz = cmsis::dap::CLOCK_KHZ_DEFAULT * 1000U;

// Delay-loop iterations that make up one microsecond, used by DAP_Delay().
// One PIN_DELAY per half period, so at f kHz a half period is 500/f us and
// costs (Speed_Coeff/f - Speed_Offset) iterations; iterations per microsecond
// is that divided by 500/f, i.e. (Speed_Coeff - Speed_Offset*f)/500, whose
// limit at low clocks -- the only regime where a us-scale delay is meaningful
// -- is Speed_Coeff/500.
static inline uint32_t Delay_Iterations_Per_us(void) {
  uint32_t n = (Speed_Coeff + 250U) / 500U;
  return (n != 0U) ? n : 1U;
}

// Common clock delay calculation routine
//   clock:    requested SWJ frequency in Hertz
//
// This replaces upstream CMSIS-DAP's CPU_CLOCK / IO_PORT_WRITE_CYCLES /
// DELAY_SLOW_CYCLES model. That model is the same formula rearranged --
//   Speed_Coeff  == CPU_CLOCK / (2000 * DELAY_SLOW_CYCLES)
//   Speed_Offset == IO_PORT_WRITE_CYCLES / DELAY_SLOW_CYCLES
// -- but expressed through a nominal CPU clock and per-instruction cycle
// counts, none of which can be measured directly on a Linux userspace bitbang.
// The two numbers here can be, by clocking the line and comparing.
static void Set_Clock_Delay(uint32_t clock) {
  Current_Clock_Hz = clock;

  // Round the requested frequency down, not up: a higher khz means a shorter
  // delay, so rounding up would clock the target faster than it was asked for.
  // OpenOCD's own interface is kHz-granular, so this only ever bites on a
  // request it cannot express anyway.
  uint32_t khz = clock / 1000U;
  if (khz == 0U) {
    khz = 1U;
  }

  uint32_t iterations = (Speed_Coeff + khz - 1U) / khz;   // DIV_ROUND_UP
  if (iterations <= Speed_Offset) {
    // At or above the fastest clock the bitbang can produce: no delay left to
    // remove, so run flat out and let SWD_TransferFast() skip the loop
    // entirely rather than calling it with a zero count.
    DAP_Data.fast_clock  = 1U;
    DAP_Data.clock_delay = 0U;
  } else {
    DAP_Data.fast_clock  = 0U;
    DAP_Data.clock_delay = iterations - Speed_Offset;
  }
}


// Get DAP Information
//   id:      info identifier
//   info:    pointer to info data
//   return:  number of bytes in info data
static uint8_t DAP_Info(uint8_t id, uint8_t *info) {
  uint8_t length = 0U;

  switch (id) {
    case DAP_ID_VENDOR:
      length = DAP_GetVendorString((char *)info);
      break;
    case DAP_ID_PRODUCT:
      length = DAP_GetProductString((char *)info);
      break;
    case DAP_ID_SER_NUM:
      length = DAP_GetSerNumString((char *)info);
      break;
    case DAP_ID_DAP_FW_VER:
      // Piggy-back the currently selected SWD mux position onto the FW
      // version string instead of a dedicated DAP_Info id: OpenOCD's
      // cmsis-dap driver already queries and LOG_INFO()s this field itself,
      // both unprompted on every connect (cmsis_dap_get_version_info(), see
      // cmsis_dap.c) and on demand via the "cmsis-dap info" TCL command --
      // so this needs no changes on the OpenOCD side to become visible.
      // With --no-swd-mux there is no position to report, so the string stays
      // bare rather than advertising a mux that is not there.
      if (!swdmux::is_enabled()) {
        length = (uint8_t)snprintf((char *)info, 60, "%s", DAP_FW_VER);
      } else if (swdmux::current_pos() < 0) {
        length = (uint8_t)snprintf((char *)info, 60, "%s swdpos=?", DAP_FW_VER);
      } else {
        length = (uint8_t)snprintf((char *)info, 60, "%s swdpos=%d", DAP_FW_VER, swdmux::current_pos());
      }
      length += 1U; // include terminating NUL, like the other *String() helpers
      break;
    case DAP_ID_DEVICE_VENDOR:
      length = DAP_GetTargetDeviceVendorString((char *)info);
      break;
    case DAP_ID_DEVICE_NAME:
      length = DAP_GetTargetDeviceNameString((char *)info);
      break;
    case DAP_ID_BOARD_VENDOR:
      length = DAP_GetTargetBoardVendorString((char *)info);
      break;
    case DAP_ID_BOARD_NAME:
      length = DAP_GetTargetBoardNameString((char *)info);
      break;
    case DAP_ID_PRODUCT_FW_VER:
      length = DAP_GetProductFirmwareVersionString((char *)info);
      break;
    case DAP_ID_CAPABILITIES:
      info[0] = ((DAP_SWD  != 0)         ? (1U << 0) : 0U) |
                ((DAP_JTAG != 0)         ? (1U << 1) : 0U) |
                ((SWO_UART != 0)         ? (1U << 2) : 0U) |
                ((SWO_MANCHESTER != 0)   ? (1U << 3) : 0U) |
                /* Atomic Commands  */     (1U << 4)       |
                ((TIMESTAMP_CLOCK != 0U) ? (1U << 5) : 0U) |
                ((SWO_STREAM != 0U)      ? (1U << 6) : 0U) |
                ((DAP_UART != 0U)        ? (1U << 7) : 0U);

      info[1] = ((DAP_UART_USB_COM_PORT != 0) ? (1U << 0) : 0U);
      length = 2U;
      break;
    case DAP_ID_TIMESTAMP_CLOCK:
#if (TIMESTAMP_CLOCK != 0U)
      info[0] = (uint8_t)(TIMESTAMP_CLOCK >>  0);
      info[1] = (uint8_t)(TIMESTAMP_CLOCK >>  8);
      info[2] = (uint8_t)(TIMESTAMP_CLOCK >> 16);
      info[3] = (uint8_t)(TIMESTAMP_CLOCK >> 24);
      length = 4U;
#endif
      break;
    case DAP_ID_UART_RX_BUFFER_SIZE:
      break;
    case DAP_ID_UART_TX_BUFFER_SIZE:
      break;
    case DAP_ID_SWO_BUFFER_SIZE:
      break;
    case DAP_ID_PACKET_SIZE:
      info[0] = (uint8_t)(DAP_PACKET_SIZE >> 0);
      info[1] = (uint8_t)(DAP_PACKET_SIZE >> 8);
      length = 2U;
      break;
    case DAP_ID_PACKET_COUNT:
      info[0] = DAP_PACKET_COUNT;
      length = 1U;
      break;
    default:
      break;
  }

  return (length);
}

// Process Delay command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_Delay(const uint8_t *request, uint8_t *response) {
  uint32_t delay;

  delay  = (uint32_t)(*(request+0)) |
           (uint32_t)(*(request+1) << 8);
  delay *= Delay_Iterations_Per_us();

  PIN_DELAY_SLOW(delay);

  *response = DAP_OK;
  return ((2U << 16) | 1U);
}


// Process Host Status command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_HostStatus(const uint8_t *request, uint8_t *response) {

  switch (*request) {
    case DAP_DEBUGGER_CONNECTED:
      //is 1 when the DAP hardware is connected to a debugger
      break;
    case DAP_TARGET_RUNNING:
      // uint32_t state = (*(request+1) & 1U);
      // 1: program execution in target started.
      // 0: program execution in target stopped.
      break;
    default:
      *response = DAP_ERROR;
      return ((2U << 16) | 1U);
  }

  *response = DAP_OK;
  return ((2U << 16) | 1U);
}


// Process Connect command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_Connect(const uint8_t *request, uint8_t *response) {
  uint32_t port;

  if (*request == DAP_PORT_AUTODETECT) {
    port = DAP_DEFAULT_PORT;
  } else {
    port = *request;
  }

  // Whatever the client cached about the DP and the AP, and whatever we found
  // in target memory, describes a session that is over.
  swd::shadow_reset();
  rtt::reset();
  rp2040::reset();

  switch (port) {
    case DAP_PORT_SWD:
      DAP_Data.debug_port = DAP_PORT_SWD;
      PORT_SWD_SETUP();
      break;
    default:
      port = DAP_PORT_DISABLED;
      break;
  }

  *response = (uint8_t)port;
  return ((1U << 16) | 1U);
}


// Process Disconnect command and prepare response
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_Disconnect(uint8_t *response) {

  DAP_Data.debug_port = DAP_PORT_DISABLED;
  PORT_OFF();
  swd::shadow_reset();
  rtt::reset();
  rp2040::reset();

  *response = DAP_OK;
  return (1U);
}


// Process Reset Target command and prepare response
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_ResetTarget(uint8_t *response) {

  *(response+1) = RESET_TARGET();
  *(response+0) = DAP_OK;
  return (2U);
}


// Process SWJ Pins command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_SWJ_Pins(const uint8_t *request, uint8_t *response) {
  uint32_t value;
  uint32_t select;
  uint32_t wait;
  uint32_t timestamp;

  value  = (uint32_t) *(request+0);
  select = (uint32_t) *(request+1);
  wait   = (uint32_t)(*(request+2) <<  0) |
           (uint32_t)(*(request+3) <<  8) |
           (uint32_t)(*(request+4) << 16) |
           (uint32_t)(*(request+5) << 24);

  if ((select & (1U << DAP_SWJ_SWCLK_TCK)) != 0U) {
    if ((value & (1U << DAP_SWJ_SWCLK_TCK)) != 0U) {
      PIN_SWCLK_TCK_SET();
    } else {
      PIN_SWCLK_TCK_CLR();
    }
  }
  if ((select & (1U << DAP_SWJ_SWDIO_TMS)) != 0U) {
    if ((value & (1U << DAP_SWJ_SWDIO_TMS)) != 0U) {
      PIN_SWDIO_TMS_SET();
    } else {
      PIN_SWDIO_TMS_CLR();
    }
  }
  if ((select & (1U << DAP_SWJ_TDI)) != 0U) {
    PIN_TDI_OUT(value >> DAP_SWJ_TDI);
  }
  if ((select & (1U << DAP_SWJ_nTRST)) != 0U) {
    PIN_nTRST_OUT(value >> DAP_SWJ_nTRST);
  }
  if ((select & (1U << DAP_SWJ_nRESET)) != 0U){
    PIN_nRESET_OUT(value >> DAP_SWJ_nRESET);
  }

  if (wait != 0U) {
#if (TIMESTAMP_CLOCK != 0U)
    if (wait > 3000000U) {
      wait = 3000000U;
    }
#if (TIMESTAMP_CLOCK >= 1000000U)
    wait *= TIMESTAMP_CLOCK / 1000000U;
#else
    wait /= 1000000U / TIMESTAMP_CLOCK;
#endif
#else
    wait  = 1U;
#endif
    timestamp = TIMESTAMP_GET();
    do {
      if ((select & (1U << DAP_SWJ_SWCLK_TCK)) != 0U) {
        if ((value >> DAP_SWJ_SWCLK_TCK) ^ PIN_SWCLK_TCK_IN()) {
          continue;
        }
      }
      if ((select & (1U << DAP_SWJ_SWDIO_TMS)) != 0U) {
        if ((value >> DAP_SWJ_SWDIO_TMS) ^ PIN_SWDIO_TMS_IN()) {
          continue;
        }
      }
      if ((select & (1U << DAP_SWJ_TDI)) != 0U) {
        if ((value >> DAP_SWJ_TDI) ^ PIN_TDI_IN()) {
          continue;
        }
      }
      if ((select & (1U << DAP_SWJ_nTRST)) != 0U) {
        if ((value >> DAP_SWJ_nTRST) ^ PIN_nTRST_IN()) {
          continue;
        }
      }
      if ((select & (1U << DAP_SWJ_nRESET)) != 0U) {
        if ((value >> DAP_SWJ_nRESET) ^ PIN_nRESET_IN()) {
          continue;
        }
      }
      break;
    } while ((TIMESTAMP_GET() - timestamp) < wait);
  }

  value = (PIN_SWCLK_TCK_IN() << DAP_SWJ_SWCLK_TCK) |
          (PIN_SWDIO_TMS_IN() << DAP_SWJ_SWDIO_TMS) |
          (PIN_TDI_IN()       << DAP_SWJ_TDI)       |
          (PIN_TDO_IN()       << DAP_SWJ_TDO)       |
          (PIN_nTRST_IN()     << DAP_SWJ_nTRST)     |
          (PIN_nRESET_IN()    << DAP_SWJ_nRESET);

  *response = (uint8_t)value;

  return ((6U << 16) | 1U);
}


// Process SWJ Clock command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_SWJ_Clock(const uint8_t *request, uint8_t *response) {
  uint32_t clock;

  clock = (uint32_t)(*(request+0) <<  0) |
          (uint32_t)(*(request+1) <<  8) |
          (uint32_t)(*(request+2) << 16) |
          (uint32_t)(*(request+3) << 24);

  Set_Clock_Delay(clock);

  *response = DAP_OK;
  return ((4U << 16) | 1U);
}


// Process SWJ Sequence command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_SWJ_Sequence(const uint8_t *request, uint8_t *response) {
  uint32_t count;

  count = *request++;
  if (count == 0U) {
    count = 256U;
  }

  SWJ_Sequence(count, request);
  *response = DAP_OK;

  count = (count + 7U) >> 3;

  return (((count + 1U) << 16) | 1U);
}


// Process SWD Configure command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_SWD_Configure(const uint8_t *request, uint8_t *response) {
  uint8_t value;

  value = *request;
  DAP_Data.swd_conf.turnaround = (value & 0x03U) + 1U;
  DAP_Data.swd_conf.data_phase = (value & 0x04U) ? 1U : 0U;

  *response = DAP_OK;

  return ((1U << 16) | 1U);
}


// Process SWD Sequence command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_SWD_Sequence(const uint8_t *request, uint8_t *response) {
  uint32_t sequence_info;
  uint32_t sequence_count;
  uint32_t request_count;
  uint32_t response_count;
  uint32_t count;

  *response++ = DAP_OK;

  request_count  = 1U;
  response_count = 1U;

  sequence_count = *request++;
  while (sequence_count--) {
    sequence_info = *request++;
    count = sequence_info & SWD_SEQUENCE_CLK;
    if (count == 0U) {
      count = 64U;
    }
    count = (count + 7U) / 8U;

    if ((sequence_info & SWD_SEQUENCE_DIN) != 0U) {
      PIN_SWDIO_OUT_DISABLE();
    } else {
      PIN_SWDIO_OUT_ENABLE();
    }
    SWD_Sequence(sequence_info, request, response);
    if (sequence_count == 0U) {
      PIN_SWDIO_OUT_ENABLE();
    }

    if ((sequence_info & SWD_SEQUENCE_DIN) != 0U) {
      request_count++;
      response += count;
      response_count += count;
    } else {
      request += count;
      request_count += count + 1U;
    }
  }

  return ((request_count << 16) | response_count);
}


// Process JTAG Sequence command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_JTAG_Sequence(const uint8_t *request, uint8_t *response) {
  uint32_t sequence_info;
  uint32_t sequence_count;
  uint32_t request_count;
  uint32_t response_count;
  uint32_t count;

  *response++ = DAP_ERROR;
  request_count  = 1U;
  response_count = 1U;

  sequence_count = *request++;
  while (sequence_count--) {
    sequence_info = *request++;
    count = sequence_info & JTAG_SEQUENCE_TCK;
    if (count == 0U) {
      count = 64U;
    }
    count = (count + 7U) / 8U;

    request += count;
    request_count += count + 1U;
  }

  return ((request_count << 16) | response_count);
}


// Process JTAG Configure command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_JTAG_Configure(const uint8_t *request, uint8_t *response) {
  uint32_t count;
  count = *request;
  *response = DAP_ERROR;
  return (((count + 1U) << 16) | 1U);
}


// Process JTAG IDCODE command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_JTAG_IDCode(const uint8_t *request, uint8_t *response) {
  (void)request;
  *response = DAP_ERROR;
  return ((1U << 16) | 1U);
}


// Process Transfer Configure command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_TransferConfigure(const uint8_t *request, uint8_t *response) {

  DAP_Data.transfer.idle_cycles =            *(request+0);
  DAP_Data.transfer.retry_count = (uint16_t) *(request+1) |
                                  (uint16_t)(*(request+2) << 8);
  DAP_Data.transfer.match_retry = (uint16_t) *(request+3) |
                                  (uint16_t)(*(request+4) << 8);

  *response = DAP_OK;
  return ((5U << 16) | 1U);
}


// Process SWD Transfer command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_SWD_Transfer(const uint8_t *request, uint8_t *response) {
  const
  uint8_t  *request_head;
  uint32_t  request_count;
  uint32_t  request_value;
  uint8_t  *response_head;
  uint32_t  response_count;
  uint32_t  response_value;
  uint32_t  post_read;
  uint32_t  check_write;
  uint32_t  match_value;
  uint32_t  match_retry;
  uint32_t  retry;
  uint32_t  data;
#if (TIMESTAMP_CLOCK != 0U)
  uint32_t  timestamp;
#endif

  request_head   = request;

  response_count = 0U;
  response_value = 0U;
  response_head  = response;
  response      += 2;

  DAP_TransferAbort = 0U;

  post_read   = 0U;
  check_write = 0U;

  request++;            // Ignore DAP index

  request_count = *request++;

  while (request_count != 0) {
    request_count--;
    request_value = *request++;
    if ((request_value & DAP_TRANSFER_RnW) != 0U) {
      // Read register
      if (post_read) {
        // Read was posted before
        retry = DAP_Data.transfer.retry_count;
        if ((request_value & (DAP_TRANSFER_APnDP | DAP_TRANSFER_MATCH_VALUE)) == DAP_TRANSFER_APnDP) {
          // Read previous AP data and post next AP read
          do {
            response_value = SWD_Transfer(request_value, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
        } else {
          // Read previous AP data
          do {
            response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          post_read = 0U;
        }
        if (response_value != DAP_TRANSFER_OK) {
          break;
        }
        // Store previous AP data
        *response++ = (uint8_t) data;
        *response++ = (uint8_t)(data >>  8);
        *response++ = (uint8_t)(data >> 16);
        *response++ = (uint8_t)(data >> 24);
#if (TIMESTAMP_CLOCK != 0U)
        if (post_read) {
          // Store Timestamp of next AP read
          if ((request_value & DAP_TRANSFER_TIMESTAMP) != 0U) {
            timestamp = DAP_Data.timestamp;
            *response++ = (uint8_t) timestamp;
            *response++ = (uint8_t)(timestamp >>  8);
            *response++ = (uint8_t)(timestamp >> 16);
            *response++ = (uint8_t)(timestamp >> 24);
          }
        }
#endif
      }
      if ((request_value & DAP_TRANSFER_MATCH_VALUE) != 0U) {
        // Read with value match
        match_value = (uint32_t)(*(request+0) <<  0) |
                      (uint32_t)(*(request+1) <<  8) |
                      (uint32_t)(*(request+2) << 16) |
                      (uint32_t)(*(request+3) << 24);
        request += 4;
        match_retry = DAP_Data.transfer.match_retry;
        if ((request_value & DAP_TRANSFER_APnDP) != 0U) {
          // Post AP read
          retry = DAP_Data.transfer.retry_count;
          do {
            response_value = SWD_Transfer(request_value, NULL);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          if (response_value != DAP_TRANSFER_OK) {
            break;
          }
        }
        do {
          // Read register until its value matches or retry counter expires
          retry = DAP_Data.transfer.retry_count;
          do {
            response_value = SWD_Transfer(request_value, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          if (response_value != DAP_TRANSFER_OK) {
            break;
          }
        } while (((data & DAP_Data.transfer.match_mask) != match_value) && match_retry-- && !DAP_TransferAbort);
        if ((data & DAP_Data.transfer.match_mask) != match_value) {
          response_value |= DAP_TRANSFER_MISMATCH;
        }
        if (response_value != DAP_TRANSFER_OK) {
          break;
        }
      } else {
        // Normal read
        retry = DAP_Data.transfer.retry_count;
        if ((request_value & DAP_TRANSFER_APnDP) != 0U) {
          // Read AP register
          if (post_read == 0U) {
            // Post AP read
            do {
              response_value = SWD_Transfer(request_value, NULL);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
            if (response_value != DAP_TRANSFER_OK) {
              break;
            }
#if (TIMESTAMP_CLOCK != 0U)
            // Store Timestamp
            if ((request_value & DAP_TRANSFER_TIMESTAMP) != 0U) {
              timestamp = DAP_Data.timestamp;
              *response++ = (uint8_t) timestamp;
              *response++ = (uint8_t)(timestamp >>  8);
              *response++ = (uint8_t)(timestamp >> 16);
              *response++ = (uint8_t)(timestamp >> 24);
            }
#endif
            post_read = 1U;
          }
        } else {
          // Read DP register
          do {
            response_value = SWD_Transfer(request_value, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          if (response_value != DAP_TRANSFER_OK) {
            break;
          }
#if (TIMESTAMP_CLOCK != 0U)
          // Store Timestamp
          if ((request_value & DAP_TRANSFER_TIMESTAMP) != 0U) {
            timestamp = DAP_Data.timestamp;
            *response++ = (uint8_t) timestamp;
            *response++ = (uint8_t)(timestamp >>  8);
            *response++ = (uint8_t)(timestamp >> 16);
            *response++ = (uint8_t)(timestamp >> 24);
          }
#endif
          // Store data
          *response++ = (uint8_t) data;
          *response++ = (uint8_t)(data >>  8);
          *response++ = (uint8_t)(data >> 16);
          *response++ = (uint8_t)(data >> 24);
        }
      }
      check_write = 0U;
    } else {
      // Write register
      if (post_read) {
        // Read previous data
        retry = DAP_Data.transfer.retry_count;
        do {
          response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
        } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
        if (response_value != DAP_TRANSFER_OK) {
          break;
        }
        // Store previous data
        *response++ = (uint8_t) data;
        *response++ = (uint8_t)(data >>  8);
        *response++ = (uint8_t)(data >> 16);
        *response++ = (uint8_t)(data >> 24);
        post_read = 0U;
      }
      // Load data
      data = (uint32_t)(*(request+0) <<  0) |
             (uint32_t)(*(request+1) <<  8) |
             (uint32_t)(*(request+2) << 16) |
             (uint32_t)(*(request+3) << 24);
      request += 4;
      if ((request_value & DAP_TRANSFER_MATCH_MASK) != 0U) {
        // Write match mask
        DAP_Data.transfer.match_mask = data;
        response_value = DAP_TRANSFER_OK;
      } else {
        // Write DP/AP register
        retry = DAP_Data.transfer.retry_count;
        do {
          response_value = SWD_Transfer(request_value, &data);
        } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
        if (response_value != DAP_TRANSFER_OK) {
          break;
        }
#if (TIMESTAMP_CLOCK != 0U)
        // Store Timestamp
        if ((request_value & DAP_TRANSFER_TIMESTAMP) != 0U) {
          timestamp = DAP_Data.timestamp;
          *response++ = (uint8_t) timestamp;
          *response++ = (uint8_t)(timestamp >>  8);
          *response++ = (uint8_t)(timestamp >> 16);
          *response++ = (uint8_t)(timestamp >> 24);
        }
#endif
        check_write = 1U;
      }
    }
    response_count++;
    if (DAP_TransferAbort) {
      break;
    }
  }

  while (request_count != 0) {
    // Process canceled requests
    request_count--;
    request_value = *request++;
    if ((request_value & DAP_TRANSFER_RnW) != 0U) {
      // Read register
      if ((request_value & DAP_TRANSFER_MATCH_VALUE) != 0U) {
        // Read with value match
        request += 4;
      }
    } else {
      // Write register
      request += 4;
    }
  }

  if (response_value == DAP_TRANSFER_OK) {
    if (post_read) {
      // Read previous data
      retry = DAP_Data.transfer.retry_count;
      do {
        response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) {
        goto end;
      }
      // Store previous data
      *response++ = (uint8_t) data;
      *response++ = (uint8_t)(data >>  8);
      *response++ = (uint8_t)(data >> 16);
      *response++ = (uint8_t)(data >> 24);
    } else if (check_write) {
      // Check last write
      retry = DAP_Data.transfer.retry_count;
      do {
        response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
    }
  }

end:
  *(response_head+0) = (uint8_t)response_count;
  *(response_head+1) = (uint8_t)response_value;

  return (((uint32_t)(request - request_head) << 16) | (uint32_t)(response - response_head));
}

// Process Dummy Transfer command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_Dummy_Transfer(const uint8_t *request, uint8_t *response) {
  const
  uint8_t  *request_head;
  uint32_t  request_count;
  uint32_t  request_value;

  request_head  =  request;

  request++;            // Ignore DAP index

  request_count = *request++;

  for (; request_count != 0U; request_count--) {
    // Process dummy requests
    request_value = *request++;
    if ((request_value & DAP_TRANSFER_RnW) != 0U) {
      // Read register
      if ((request_value & DAP_TRANSFER_MATCH_VALUE) != 0U) {
        // Read with value match
        request += 4;
      }
    } else {
      // Write register
      request += 4;
    }
  }

  *(response+0) = 0U;   // Response count
  *(response+1) = 0U;   // Response value

  return (((uint32_t)(request - request_head) << 16) | 2U);
}


// Process Transfer command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_Transfer(const uint8_t *request, uint8_t *response) {
  uint32_t num;

  switch (DAP_Data.debug_port) {
    case DAP_PORT_SWD:
      num = DAP_SWD_Transfer(request, response);
      break;
    default:
      num = DAP_Dummy_Transfer(request, response);
      break;
  }

  return (num);
}


// Process SWD Transfer Block command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_SWD_TransferBlock(const uint8_t *request, uint8_t *response) {
  uint32_t  request_count;
  uint32_t  request_value;
  uint32_t  response_count;
  uint32_t  response_value;
  uint8_t  *response_head;
  uint32_t  retry;
  uint32_t  data;

  response_count = 0U;
  response_value = 0U;
  response_head  = response;
  response      += 3;

  DAP_TransferAbort = 0U;

  request++;            // Ignore DAP index

  request_count = (uint32_t)(*(request+0) << 0) |
                  (uint32_t)(*(request+1) << 8);
  request += 2;
  if (request_count == 0U) {
    goto end;
  }

  request_value = *request++;
  if ((request_value & DAP_TRANSFER_RnW) != 0U) {
    // Read register block
    if ((request_value & DAP_TRANSFER_APnDP) != 0U) {
      // Post AP read
      retry = DAP_Data.transfer.retry_count;
      do {
        response_value = SWD_Transfer(request_value, NULL);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) {
        goto end;
      }
    }
    while (request_count--) {
      // Read DP/AP register
      if ((request_count == 0U) && ((request_value & DAP_TRANSFER_APnDP) != 0U)) {
        // Last AP read
        request_value = DP_RDBUFF | DAP_TRANSFER_RnW;
      }
      retry = DAP_Data.transfer.retry_count;
      do {
        response_value = SWD_Transfer(request_value, &data);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) {
        goto end;
      }
      // Store data
      *response++ = (uint8_t) data;
      *response++ = (uint8_t)(data >>  8);
      *response++ = (uint8_t)(data >> 16);
      *response++ = (uint8_t)(data >> 24);
      response_count++;
    }
  } else {
    // Write register block
    while (request_count--) {
      // Load data
      data = (uint32_t)(*(request+0) <<  0) |
             (uint32_t)(*(request+1) <<  8) |
             (uint32_t)(*(request+2) << 16) |
             (uint32_t)(*(request+3) << 24);
      request += 4;
      // Write DP/AP register
      retry = DAP_Data.transfer.retry_count;
      do {
        response_value = SWD_Transfer(request_value, &data);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) {
        goto end;
      }
      response_count++;
    }
    // Check last write
    retry = DAP_Data.transfer.retry_count;
    do {
      response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
  }

end:
  *(response_head+0) = (uint8_t)(response_count >> 0);
  *(response_head+1) = (uint8_t)(response_count >> 8);
  *(response_head+2) = (uint8_t) response_value;

  return ((uint32_t)(response - response_head));
}

// Process Transfer Block command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_TransferBlock(const uint8_t *request, uint8_t *response) {
  uint32_t num;

  switch (DAP_Data.debug_port) {
    case DAP_PORT_SWD:
      num = DAP_SWD_TransferBlock (request, response);
      break;
    default:
      *(response+0) = 0U;       // Response count [7:0]
      *(response+1) = 0U;       // Response count[15:8]
      *(response+2) = 0U;       // Response value
      num = 3U;
      break;
  }

  if ((*(request+3) & DAP_TRANSFER_RnW) != 0U) {
    // Read register block
    num |=  4U << 16;
  } else {
    // Write register block
    num |= (4U + (((uint32_t)(*(request+1)) | (uint32_t)(*(request+2) << 8)) * 4)) << 16;
  }

  return (num);
}


// Process SWD Write ABORT command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_SWD_WriteAbort(const uint8_t *request, uint8_t *response) {
  uint32_t data;

  // Load data (Ignore DAP index)
  data = (uint32_t)(*(request+1) <<  0) |
         (uint32_t)(*(request+2) <<  8) |
         (uint32_t)(*(request+3) << 16) |
         (uint32_t)(*(request+4) << 24);

  // Write Abort register
  SWD_Transfer(DP_ABORT, &data);

  *response = DAP_OK;
  return (1U);
}

// Process Write ABORT command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_WriteAbort(const uint8_t *request, uint8_t *response) {
  uint32_t num;

  switch (DAP_Data.debug_port) {
    case DAP_PORT_SWD:
      num = DAP_SWD_WriteAbort (request, response);
      break;
    default:
      *response = DAP_ERROR;
      num = 1U;
      break;
  }
  return ((5U << 16) | num);
}

// How many response bytes are still unclaimed in the packet being built.
//
// The command handlers inherited from CMSIS-DAP do not bound their own output
// -- callers cope by allocating for the worst-case amplification and rejecting
// what comes out too long (see MAX_RESPONSE_AMPLIFICATION in DAP.h). A vendor
// command that returns a variable amount of data can do better than that: it
// sizes its answer to what is left, so a DAP_ExecuteCommands batch comes back
// short rather than coming back unsendable. Only the outermost command resets
// it; the ones inside a batch draw down what the earlier ones left.
// The mirror of it on the request side. The transport rejects a request
// longer than the packet size before it gets here, so counting down from that
// bounds how much of a variable-length request is really there to be read.
static size_t   Request_Room  = DAP_PACKET_SIZE;
static size_t   Response_Room = DAP_PACKET_SIZE;
static unsigned Command_Depth = 0U;

static void Rooms_Consume(uint32_t num) {
  const size_t taken = (size_t)(num >> 16);
  const size_t used  = (size_t)(num & 0xFFFFU);
  Request_Room  = (taken < Request_Room)  ? (Request_Room  - taken) : 0U;
  Response_Room = (used  < Response_Room) ? (Response_Room - used)  : 0U;
}

//**************************************************************************************************
/**
\defgroup DAP_Vendor_Adapt_gr Adapt Vendor Commands
\ingroup DAP_Vendor_gr
@{

The file DAP_vendor.c provides template source code for extension of a Debug Unit with
Vendor Commands. Copy this file to the project folder of the Debug Unit and add the
file to the MDK-ARM project under the file group Configuration.
*/

/** Process DAP Vendor Command and prepare Response Data
\param request   pointer to request data
\param response  pointer to response data
\return          number of bytes in response (lower 16 bits)
                 number of bytes in request (upper 16 bits)
*/
static uint32_t DAP_ProcessVendorCommand(const uint8_t *request, uint8_t *response) {
  uint8_t cmd = *request++;      // first byte in request is Command ID; request now points to command data

  // ID_DAP_Vendor1..ID_DAP_Vendor16, documented in rtt.h, dp_connect.h,
  // node_query.h, rp2040.h and calibrate.h.
  if (rtt::handles(cmd)) {
    return rtt::handle_command(request - 1, response, Request_Room,
                               Response_Room);
  }
  if (dp::handles(cmd)) {
    return dp::handle_command(request - 1, response, Request_Room);
  }
  if (node_query::handles(cmd)) {
    return node_query::handle_command(request - 1, response, Request_Room,
                                      Response_Room);
  }
  if (rp2040::handles(cmd)) {
    return rp2040::handle_command(request - 1, response, Request_Room);
  }
  if (calibrate::handles(cmd)) {
    return calibrate::handle_command(request - 1, response, Request_Room);
  }

  switch (cmd) {
    case ID_DAP_Vendor0: {
      // Select SWD mux position (see swdmux::select_pos()):
      //   request[0]  = position (0 = extender, 1..12 = node)
      //   response[0] = command ID echo
      //   response[1] = DAP_OK | DAP_ERROR
      // Under --no-swd-mux swdmux::select_pos() refuses, so this reports
      // DAP_ERROR rather than silently pretending the switch happened.
      *response++ = cmd;
      *response    = swdmux::select_pos(*request) ? DAP_OK : DAP_ERROR;
      // Two bytes of request, not one: the position byte is consumed as well
      // as the command ID. Reporting one would leave DAP_ExecuteCommands
      // reading every later command in the batch from the wrong offset.
      return ((2U << 16) | 2U);
    }
    default: {
      *response = ID_DAP_Invalid;
      return ((1U << 16) | 1U);
    }
  }
}

///@}

namespace cmsis {

namespace dap {

// Setup DAP
void init(unsigned int speed_coeff, unsigned int speed_offset, unsigned int clock_hz) {

  Speed_Coeff  = speed_coeff;
  Speed_Offset = speed_offset;

  // Default settings
  DAP_Data.debug_port  = 0U;
  DAP_Data.transfer.idle_cycles = 0U;
  DAP_Data.transfer.retry_count = 100U;
  DAP_Data.transfer.match_retry = 0U;
  DAP_Data.transfer.match_mask  = 0x00000000U;
  DAP_Data.swd_conf.turnaround  = 1U;
  DAP_Data.swd_conf.data_phase  = 0U;

  // Sets DAP_Data.fast_clock and DAP_Data.clock_delay.
  Set_Clock_Delay(clock_hz);

  DAP_SETUP();  // Device specific setup
}

// Release the DAP I/O pins and unmap the GPIO registers. Safe to call even
// if init() was never called.
void shutdown() {
  if (gpio::is_mapped()) {
    PORT_OFF();
  }
  gpio::deinit();
}

// Self-calibrate Speed_Coeff/Speed_Offset. See DAP.h for the contract.
//
// Set_Clock_Delay()'s model is iterations = ceil(Speed_Coeff/khz), clock_delay
// = iterations - Speed_Offset, and Delay_Iterations_Per_us's comment works out
// what that means in real time: one PIN_DELAY_SLOW() iteration costs
// 500000/Speed_Coeff nanoseconds, and Speed_Offset is the fixed cost of one
// bit transition's GPIO register write, expressed in that same iteration
// unit. Both halves are directly measurable on this machine by timing them,
// which is the whole reason this parameterisation replaced upstream
// CMSIS-DAP's CPU_CLOCK/DELAY_SLOW_CYCLES one (see Set_Clock_Delay's own
// comment) -- so invert the model to recover Speed_Coeff/Speed_Offset from
// the two measurements.
bool calibrate(unsigned int *speed_coeff_out, unsigned int *speed_offset_out) {
  if (!gpio::is_mapped()) {
    return false;
  }

  // Iteration cost: time one long run of the exact loop PIN_DELAY() drives.
  // Large enough that clock_gettime()'s own resolution and call overhead are
  // negligible next to the measured interval.
  constexpr uint32_t LOOP_ITERATIONS = 20000000U;
  struct timespec t0;
  struct timespec t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  PIN_DELAY_SLOW(LOOP_ITERATIONS);
  clock_gettime(CLOCK_MONOTONIC, &t1);
  const double loop_ns = (double)(t1.tv_sec - t0.tv_sec) * 1e9 +
                         (double)(t1.tv_nsec - t0.tv_nsec);
  const double iter_ns = loop_ns / (double)LOOP_ITERATIONS;

  // Fixed per-transition cost: time bare SWCLK register writes back to back,
  // the same PIN_SWCLK_CLR()/PIN_SWCLK_SET() pair every PIN_DELAY() call site
  // brackets, with no delay loop between them. This never drives SWDIO, so
  // it is safe whether or not anything is listening on the wire.
  constexpr uint32_t TOGGLE_PAIRS = 500000U;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  for (uint32_t i = 0; i < TOGGLE_PAIRS; i++) {
    PIN_SWCLK_CLR();
    PIN_SWCLK_SET();
  }
  clock_gettime(CLOCK_MONOTONIC, &t1);
  const double toggle_ns = (double)(t1.tv_sec - t0.tv_sec) * 1e9 +
                           (double)(t1.tv_nsec - t0.tv_nsec);
  // Two writes per pair, each the fixed cost one PIN_DELAY() call site pays.
  const double gpio_ns = toggle_ns / (double)(TOGGLE_PAIRS * 2U);

  if (!(iter_ns > 0.0)) {
    // A clock that didn't advance, or a loop optimised away underneath us;
    // either way there is nothing sane to divide by.
    return false;
  }

  long coeff  = (long)(500000.0 / iter_ns + 0.5);
  long offset = (long)(gpio_ns / iter_ns + 0.5);
  if (coeff < 1) {
    coeff = 1;
  }
  if (offset < 1) {
    // See --speed-offset's own floor in main.cpp: zero would claim an
    // unbounded maximum clock.
    offset = 1;
  }
  if (offset >= coeff) {
    offset = coeff - 1;
  }

  Speed_Coeff  = (uint32_t)coeff;
  Speed_Offset = (uint32_t)offset;
  Set_Clock_Delay(Current_Clock_Hz);

  *speed_coeff_out  = Speed_Coeff;
  *speed_offset_out = Speed_Offset;
  return true;
}

// Process DAP command request and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t dispatch_cmd(const uint8_t *request, uint8_t *response) {
  uint32_t num;

  if ((*request >= ID_DAP_Vendor0) && (*request <= ID_DAP_Vendor31)) {
    return DAP_ProcessVendorCommand(request, response);
  }

  *response++ = *request;

  switch (*request++) {
    case ID_DAP_Info:
      num = DAP_Info(*request, response+1);
      *response = (uint8_t)num;
      return ((2U << 16) + 2U + num);

    case ID_DAP_HostStatus:
      num = DAP_HostStatus(request, response);
      break;

    case ID_DAP_Connect:
      num = DAP_Connect(request, response);
      break;
    case ID_DAP_Disconnect:
      num = DAP_Disconnect(response);
      break;

    case ID_DAP_Delay:
      num = DAP_Delay(request, response);
      break;

    case ID_DAP_ResetTarget:
      num = DAP_ResetTarget(response);
      break;

    case ID_DAP_SWJ_Pins:
      num = DAP_SWJ_Pins(request, response);
      break;
    case ID_DAP_SWJ_Clock:
      num = DAP_SWJ_Clock(request, response);
      break;
    case ID_DAP_SWJ_Sequence:
      num = DAP_SWJ_Sequence(request, response);
      break;

    case ID_DAP_SWD_Configure:
      num = DAP_SWD_Configure(request, response);
      break;
    case ID_DAP_SWD_Sequence:
      num = DAP_SWD_Sequence(request, response);
      break;

    case ID_DAP_JTAG_Sequence:
      num = DAP_JTAG_Sequence(request, response);
      break;
    case ID_DAP_JTAG_Configure:
      num = DAP_JTAG_Configure(request, response);
      break;
    case ID_DAP_JTAG_IDCODE:
      num = DAP_JTAG_IDCode(request, response);
      break;

    case ID_DAP_TransferConfigure:
      num = DAP_TransferConfigure(request, response);
      break;
    case ID_DAP_Transfer:
      num = DAP_Transfer(request, response);
      break;
    case ID_DAP_TransferBlock:
      num = DAP_TransferBlock(request, response);
      break;

    case ID_DAP_WriteABORT:
      num = DAP_WriteAbort(request, response);
      break;

    default:
      *(response-1) = ID_DAP_Invalid;
      return ((1U << 16) | 1U);
  }

  return ((1U << 16) + 1U + num);
}

uint32_t process_cmd(const uint8_t *request, uint8_t *response) {
  if (Command_Depth == 0U) {
    Request_Room  = DAP_PACKET_SIZE;
    Response_Room = DAP_PACKET_SIZE;
  }

  const uint32_t num = dispatch_cmd(request, response);
  Rooms_Consume(num);
  return num;
}

// Execute DAP command (process request and prepare response).
// Understands ID_DAP_ExecuteCommands, which packs a batch of "cnt" DAP
// commands back-to-back into a single request and expects their responses
// packed back-to-back into a single response, in addition to plain single
// commands handled directly by process_cmd().
uint32_t execute_cmd(const uint8_t *request, uint8_t *response) {
  if (*request == ID_DAP_ExecuteCommands) {
    uint32_t cnt;
    uint32_t num;
    uint32_t n;

    *response++ = *request++;
    cnt = *request++;
    *response++ = (uint8_t)cnt;
    num = (2U << 16) | 2U;

    // The batch owns both packets, so the budgets are set once here and drawn
    // down by each command rather than reset by each of them.
    Request_Room  = DAP_PACKET_SIZE - 2U;
    Response_Room = DAP_PACKET_SIZE - 2U;
    Command_Depth = 1U;
    while (cnt--) {
      n = process_cmd(request, response);
      num += n;
      request  += (uint16_t)(n >> 16);
      response += (uint16_t)n;
    }
    Command_Depth = 0U;
    return num;
  }

  return process_cmd(request, response);
}


}  // namespace dap

}  // namespace cmsis
