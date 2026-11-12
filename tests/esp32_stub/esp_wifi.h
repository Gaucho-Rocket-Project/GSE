// -*- mode: c++ -*-
/*
 * esp_wifi.h  --  stand-in for the ESP-IDF Wi-Fi driver, host check only
 * ---------------------------------------------------------------------------
 * ESP-NOW rides on the Wi-Fi radio: it needs the interface up in station mode.
 * It does NOT need an association, and firmware/src/link.h never calls
 * esp_wifi_connect() -- so this stub deliberately declares no connect/scan API:
 * if that call ever creeps back in, the host check fails instead of the link
 * silently depending on a router.
 */
#ifndef ESP_WIFI_H_SYNTAX_STUB
#define ESP_WIFI_H_SYNTAX_STUB

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define ESP_WIFI_MAX_SSID_LEN 32

typedef enum {
    WIFI_MODE_NULL = 0,
    WIFI_MODE_STA,
    WIFI_MODE_AP,
    WIFI_MODE_APSTA
} wifi_mode_t;

typedef enum {
    WIFI_IF_STA = 0,
    WIFI_IF_AP
} wifi_interface_t;

typedef enum {
    WIFI_SECOND_CHAN_NONE = 0,
    WIFI_SECOND_CHAN_ABOVE,
    WIFI_SECOND_CHAN_BELOW
} wifi_second_chan_t;

typedef enum {
    WIFI_STORAGE_FLASH = 0,
    WIFI_STORAGE_RAM
} wifi_storage_t;

/* The received-packet metadata block.  Only `rssi` is used by the firmware, and
 * only on IDF 5; the real struct is a bitfield soup that differs by target
 * chip.  Keeping one member makes the stub honest about what we touch. */
typedef struct {
    signed rssi : 8;
    unsigned    : 24;
} wifi_pkt_rx_ctrl_t;

typedef struct {
    int dummy;
} wifi_init_config_t;

#define WIFI_INIT_CONFIG_DEFAULT() (wifi_init_config_t{ 0 })

esp_err_t esp_wifi_init(const wifi_init_config_t *config);
esp_err_t esp_wifi_deinit(void);
esp_err_t esp_wifi_set_mode(wifi_mode_t mode);
esp_err_t esp_wifi_get_mode(wifi_mode_t *mode);
esp_err_t esp_wifi_start(void);
esp_err_t esp_wifi_stop(void);
esp_err_t esp_wifi_set_storage(wifi_storage_t storage);
esp_err_t esp_wifi_get_mac(wifi_interface_t ifx, uint8_t mac[6]);
esp_err_t esp_wifi_set_channel(uint8_t primary, wifi_second_chan_t second);
esp_err_t esp_wifi_get_channel(uint8_t *primary, wifi_second_chan_t *second);

#endif /* ESP_WIFI_H_SYNTAX_STUB */
