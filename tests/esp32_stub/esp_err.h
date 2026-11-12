// -*- mode: c++ -*-
/* esp_err.h -- part of the ESP-IDF stand-in used by tests/esp32_syntax_check.sh.
 * Only the type and the two codes the firmware tests for.  The real header
 * defines ~200 codes; none of the rest is reachable from firmware/src/. */
#ifndef ESP_ERR_H_SYNTAX_STUB
#define ESP_ERR_H_SYNTAX_STUB

#include <stdint.h>

typedef int esp_err_t;

#define ESP_OK                    0
#define ESP_FAIL                  -1
#define ESP_ERR_NO_MEM            0x101
#define ESP_ERR_INVALID_ARG       0x102
#define ESP_ERR_INVALID_STATE     0x103
#define ESP_ERR_INVALID_SIZE      0x104
#define ESP_ERR_NOT_FOUND         0x105
#define ESP_ERR_NOT_SUPPORTED     0x106

#define ESP_ERROR_CHECK(x) do { (void)(x); } while (0)

#endif /* ESP_ERR_H_SYNTAX_STUB */
