#pragma once
#include <cstdint>
#define ESP_MAC_WIFI_STA 0
inline int esp_read_mac(uint8_t *, int) { return 1; }
