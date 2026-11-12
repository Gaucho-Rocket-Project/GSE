// -*- mode: c++ -*-
/* esp_netif.h -- part of the ESP-IDF stand-in (tests/esp32_syntax_check.sh).
 * ESP-NOW needs the network interface layer initialised even though these nodes
 * never get an IP address.  Only the one call the firmware makes is declared. */
#ifndef ESP_NETIF_H_SYNTAX_STUB
#define ESP_NETIF_H_SYNTAX_STUB
#include "esp_err.h"
esp_err_t esp_netif_init(void);
#endif
