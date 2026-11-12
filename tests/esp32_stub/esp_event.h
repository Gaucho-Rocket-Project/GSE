// -*- mode: c++ -*-
/* esp_event.h -- part of the ESP-IDF stand-in (tests/esp32_syntax_check.sh).
 * The Wi-Fi stack needs the default event loop created.  The firmware registers
 * no handlers, so none are declared here. */
#ifndef ESP_EVENT_H_SYNTAX_STUB
#define ESP_EVENT_H_SYNTAX_STUB
#include "esp_err.h"
esp_err_t esp_event_loop_create_default(void);
#endif
