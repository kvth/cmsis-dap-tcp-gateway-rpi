/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * SWD clock self-calibration. See calibrate.h for the wire format.
 */

#include "calibrate.h"

#include <string.h>

#include "DAP.h"
#include "logging.h"

namespace calibrate {

using namespace vendor;

namespace {

uint8_t *store_le32(uint8_t *p, uint32_t v) {
  *p++ = (uint8_t)v;
  *p++ = (uint8_t)(v >> 8);
  *p++ = (uint8_t)(v >> 16);
  *p++ = (uint8_t)(v >> 24);
  return p;
}

}  // namespace

bool handles(uint8_t cmd) {
  return cmd == CMD_CALIBRATE;
}

uint32_t handle_command(const uint8_t *request, uint8_t *response,
                        size_t request_room) {
  (void)request;   // no arguments beyond the command id, already consumed

  constexpr size_t REQUEST_LEN = 1U;   // id only, no arguments
  constexpr size_t BODY_LEN    = 8U;   // speed_coeff, speed_offset

  response[0] = CMD_CALIBRATE;

  if (request_room < REQUEST_LEN) {
    response[1] = STATUS_BAD_REQUEST;
    memset(response + 2, 0, BODY_LEN);
    return ((uint32_t)request_room << 16) | (uint32_t)(2U + BODY_LEN);
  }

  unsigned int speed_coeff  = 0U;
  unsigned int speed_offset = 0U;
  const bool ok = cmsis::dap::calibrate(&speed_coeff, &speed_offset);
  const Status status = ok ? STATUS_OK : STATUS_NOT_MAPPED;

  if (ok) {
    LOGI_KV("swd clock calibrated",
            "speed_coeff=%u speed_offset=%u max_khz=%u",
            speed_coeff, speed_offset, speed_coeff / speed_offset);
  } else {
    LOGW("swd clock calibration failed: gpio pins are not mapped yet");
  }

  response[1] = (uint8_t)status;
  uint8_t *body = response + 2;
  body = store_le32(body, speed_coeff);
  body = store_le32(body, speed_offset);

  return ((uint32_t)REQUEST_LEN << 16) | (uint32_t)(2U + BODY_LEN);
}

}  // namespace calibrate
