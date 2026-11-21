#pragma once
#include <cstdint>
#include <cstddef>
#define ESP_OK 0
#define ESP_ERR_ESPNOW_EXIST 1
using esp_err_t = int;
struct esp_now_peer_info_t { uint8_t peer_addr[6]; int channel; bool encrypt; };
struct esp_now_recv_info_t { const uint8_t *src_addr; };
inline esp_err_t esp_now_send(const uint8_t *, const uint8_t *, size_t) { return ESP_OK; }
inline esp_err_t esp_now_init() { return ESP_OK; }
inline esp_err_t esp_now_add_peer(esp_now_peer_info_t *) { return ESP_OK; }
inline void esp_now_register_recv_cb(void (*)(const esp_now_recv_info_t *, const uint8_t *, int)) {}
