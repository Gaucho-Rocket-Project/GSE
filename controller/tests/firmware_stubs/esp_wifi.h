#pragma once
#include <cstdint>
#define WIFI_IF_STA 0
#define WIFI_PS_NONE 0
inline int esp_wifi_get_mac(int, uint8_t *) { return 1; }
inline int esp_wifi_set_ps(int) { return 0; }
