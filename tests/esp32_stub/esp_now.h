// -*- mode: c++ -*-
/*
 * esp_now.h  --  stand-in for the ESP-IDF ESP-NOW driver, host syntax check only
 * ---------------------------------------------------------------------------
 * Mirrors the *union* of ESP-IDF 4.4 (arduino-esp32 2.x) and 5.x (3.x) for the
 * surface firmware/src/link.h uses.  The two really do differ, and the firmware
 * depends on the difference, so both are modelable from one header:
 *
 *   recv callback   4.4: void (*)(const uint8_t *mac, const uint8_t *data, int len)
 *                   5.x: void (*)(const esp_now_recv_info_t *info,
 *                                 const uint8_t *data, int len)
 *
 * Which one the firmware must implement is decided by ESP_IDF_VERSION_MAJOR
 * (see esp_idf_version.h in this directory), and the stub declares BOTH names
 * for the same type with a version guard, exactly as the real header does:
 * esp_now_register_recv_cb() takes the version-appropriate one, so a firmware
 * callback of the wrong shape is a compile error here, not a crash on a bench.
 *
 * Bodies are empty.  Nothing in this directory simulates anything.
 */
#ifndef ESP_NOW_H_SYNTAX_STUB
#define ESP_NOW_H_SYNTAX_STUB

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_wifi.h"

#define ESP_NOW_MAX_DATA_LEN          250
#define ESP_NOW_ETH_ALEN              6
#define ESP_NOW_KEY_LEN               16
#define ESP_ERR_ESPNOW_BASE           0x3060
#define ESP_ERR_ESPNOW_EXIST          (ESP_ERR_ESPNOW_BASE + 7)

typedef enum {
    ESP_NOW_SEND_SUCCESS = 0,
    ESP_NOW_SEND_FAIL    = 1
} esp_now_send_status_t;

typedef struct esp_now_peer_info {
    uint8_t          peer_addr[ESP_NOW_ETH_ALEN];
    uint8_t          lmk[ESP_NOW_KEY_LEN];
    uint8_t          channel;
    wifi_interface_t ifidx;
    bool             encrypt;
    void            *priv;
} esp_now_peer_info_t;

typedef struct esp_now_recv_info {
    uint8_t           *src_addr;
    uint8_t           *des_addr;
    wifi_pkt_rx_ctrl_t *rx_ctrl;
} esp_now_recv_info_t;

#if ESP_IDF_VERSION_MAJOR >= 5
typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t *info,
                                  const uint8_t *data, int data_len);
#else
typedef void (*esp_now_recv_cb_t)(const uint8_t *mac_addr,
                                  const uint8_t *data, int data_len);
#endif
typedef void (*esp_now_send_cb_t)(const uint8_t *mac_addr,
                                  esp_now_send_status_t status);

esp_err_t esp_now_init(void);
esp_err_t esp_now_deinit(void);
esp_err_t esp_now_register_send_cb(esp_now_send_cb_t cb);
esp_err_t esp_now_unregister_send_cb(void);
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb);
esp_err_t esp_now_unregister_recv_cb(void);
esp_err_t esp_now_send(const uint8_t *peer_addr, const uint8_t *data, size_t len);
esp_err_t esp_now_add_peer(const esp_now_peer_info_t *peer);
esp_err_t esp_now_del_peer(const uint8_t *peer_addr);
esp_err_t esp_now_set_pmk(const uint8_t *pmk);

#endif /* ESP_NOW_H_SYNTAX_STUB */
