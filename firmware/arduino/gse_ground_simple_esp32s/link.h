/*
 * link_simple.h  --  GSE link over the proven ESP-NOW ping/pong setup
 * ---------------------------------------------------------------------------
 * A drop-in replacement for link.h (same GseLink class and method set), used
 * by the "simple" Arduino sketches that tools/make_arduino_sketches.py builds:
 *
 *      gse_ground_simple_esp32s   ground board: tx_platform_esp32.cpp + gse_tx_core.c
 *      gse_vehicle_simple_esp32s  vehicle:      pad_node.cpp
 *
 * Everything above this file -- the GUI protocol frames, the ground board's
 * stale-command drop and SAFE stream, the vehicle's watchdog, failsafe,
 * servos and one-shot igniter -- is the unchanged GSE code.  Only the radio
 * setup differs from link.h, and it is copied from the ESP-NOW ping/pong test
 * sketches (firmware/espnow_ping/) that were shown to work on the bench:
 *
 *      WiFi.mode(WIFI_STA) + WiFi.disconnect(), fixed channel (GSE_NOW_CHANNEL,
 *      6 in the simple sketches), NO encryption, unicast to the one peer MAC
 *      from config.h (GSE_NOW_PAD_MAC / GSE_NOW_TX_MAC); packets from any other
 *      MAC are dropped.
 *
 * The ping/pong test is built in.  The ground board sends a 16-byte "ROCK"
 * PING (same format as the test sketches) every second; the vehicle answers
 * with a PONG; the ground board prints one line per PONG on its USB port:
 *
 *      LOG PONG seq=12 rtt=4 ms
 *      LOG PING seq=13: no PONG from the vehicle
 *
 * which the GUI's serial monitor shows.  GSE frames (7E ...) and ROCK packets
 * share the link and are told apart by size + magic.
 *
 * Without encryption, anything that knows the vehicle's MAC and channel can
 * send it ESP-NOW packets; the source-MAC filter, the arm code and the
 * hardware arm key are the remaining controls.  Use for the bench, and move
 * to link.h (CCMP) once that link is working.
 */
#ifndef GSE_LINK_H
#define GSE_LINK_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

#include "config.h"
#include "gse_protocol.h"

#if defined(GSE_LINK_LORA)
#error "link_simple.h is an ESP-NOW transport; build the simple sketches without GSE_LINK_LORA"
#endif

#if defined(GSE_ROLE_PAD)
#  define GSE_LINK_IS_PAD 1
#else
#  define GSE_LINK_IS_PAD 0
#endif

namespace gse_simple {
constexpr uint32_t kMagic = 0x524F434B;          /* "ROCK", as in the test sketches */
constexpr uint8_t  kVersion = 1;
constexpr uint8_t  kTypePing = 1;
constexpr uint8_t  kTypePong = 2;
constexpr uint32_t kPingIntervalMs = 1000;

struct __attribute__((packed)) RockPacket {
    uint32_t magic;
    uint8_t  version;
    uint8_t  type;
    uint16_t reserved;
    uint32_t sequence;
    uint32_t pingSentAtMs;
};
static_assert(sizeof(RockPacket) == 16, "ROCK packet must stay 16 bytes");
}  // namespace gse_simple

class GseLink;
static GseLink *s_link = nullptr;
static portMUX_TYPE s_rxMux = portMUX_INITIALIZER_UNLOCKED;

#if ESP_IDF_VERSION_MAJOR >= 5
static void gse_simple_recv_shim(const esp_now_recv_info_t *info,
                                 const uint8_t *data, int len);
#else
static void gse_simple_recv_shim(const uint8_t *src, const uint8_t *data, int len);
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void gse_simple_send_shim(const esp_now_send_info_t *info,
                                 esp_now_send_status_t status);
#else
static void gse_simple_send_shim(const uint8_t *mac, esp_now_send_status_t status);
#endif

class GseLink {
public:
    enum { RX_CAP = 256, MAC_LEN = 6 };

    void begin();
    void poll();                               /* ping/pong bookkeeping */

    const char *transport() const { return "espnow"; }
    String describe() const;
    String provision(uint8_t, uint8_t, uint16_t, uint16_t, uint16_t);

    /* frame = bare 7E frame (LoRa addressing arguments unused). */
    size_t send(uint16_t, uint8_t, const uint8_t *frame, size_t len)
    {
        return transmit(frame, len);
    }
    /* pkt = [3-byte LoRa prefix] + 7E frame, as gse_tx_core.c builds it. */
    size_t sendRaw(const uint8_t *pkt, size_t len)
    {
        return len <= 3 ? 0 : transmit(pkt + 3, len - 3);
    }

    int available() const
    {
        portENTER_CRITICAL(&s_rxMux);
        int n = (int)((uint8_t)(rxTail - rxHead));
        portEXIT_CRITICAL(&s_rxMux);
        return n;
    }
    int read()
    {
        portENTER_CRITICAL(&s_rxMux);
        int b = -1;
        if (rxHead != rxTail) { b = rxBuf[rxHead]; rxHead = (uint8_t)(rxHead + 1); }
        portEXIT_CRITICAL(&s_rxMux);
        return b;
    }

    bool linkUp() const { return ready; }
    const char *linkState() const
    {
        if (!started)        return "radio failed";
        if (!peerConfigured) return "peer MAC NOT CONFIGURED";
        if (!ready)          return "peer registration failed";
        return "ready (plain ESP-NOW, no encryption)";
    }
    int switchMode() const { return -1; }     /* LoRa-only query; never used here */

    /* ---- called from the Wi-Fi task ---- */
    void onDatagram(const uint8_t *src, const uint8_t *data, int len);
    void onSendStatus(bool ok) { if (ok) txOk = txOk + 1; else txFailed = txFailed + 1; }

    /* counters (diagnostics) */
    volatile uint32_t txOk = 0, txFailed = 0, rxForeign = 0, rxDropped = 0;

private:
    size_t transmit(const uint8_t *data, size_t len);
    void   push(const uint8_t *data, size_t len);
    void   sendRock(uint8_t type, uint32_t seq, uint32_t sentAt);
    static bool isPlaceholder(const uint8_t *m)
    {
        return m[0] == 0x02 && m[1] == 0 && m[2] == 0 && m[3] == 0 && m[4] == 0 &&
               (m[5] == 1 || m[5] == 2);
    }
    static String macStr(const uint8_t *m)
    {
        char b[18];
        snprintf(b, sizeof b, "%02X:%02X:%02X:%02X:%02X:%02X",
                 m[0], m[1], m[2], m[3], m[4], m[5]);
        return String(b);
    }

    uint8_t rxBuf[RX_CAP];
    volatile uint8_t rxHead = 0, rxTail = 0;
    uint8_t peerMac[MAC_LEN] = {0};
    uint8_t localMac[MAC_LEN] = {0};
    bool started = false, peerConfigured = false, ready = false;
    const char *initFault = "not started";

    /* ping/pong (fields written by the Wi-Fi task are volatile) */
    uint32_t pingSeq = 0, pingSentAt = 0, lastPingAt = 0;
    bool     pingAnswered = true;
    volatile bool     pongArrived = false;     /* ground: a PONG came back   */
    volatile uint32_t pongSeq = 0, pongSentAt = 0;
    volatile bool     pingArrived = false;     /* vehicle: a PING came in    */
    volatile uint32_t pingInSeq = 0, pingInSentAt = 0;
};

/* ----------------------------------------------------------------- code */
inline String GseLink::describe() const
{
    String s = String("ESP-NOW plain (ping/pong settings) ch ") +
               String((int)GSE_NOW_CHANNEL) + ", local " + macStr(localMac);
    if (!peerConfigured)
        return s + ", peer MAC NOT CONFIGURED (set GSE_NOW_" +
               (GSE_LINK_IS_PAD ? "TX" : "PAD") + "_MAC and rebuild)";
    return s + " -> peer " + macStr(peerMac) + ", no encryption";
}

inline String GseLink::provision(uint8_t, uint8_t, uint16_t, uint16_t, uint16_t)
{
    if (!started)        return String(initFault);
    if (!peerConfigured) return String("peer MAC not configured in config.h");
    if (!ready)          return String("esp_now_add_peer failed");
    return String();
}

inline void GseLink::begin()
{
    s_link = this;

    /* Exactly the ping/pong sketches' radio setup. */
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_wifi_set_channel((uint8_t)GSE_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        initFault = "Wi-Fi channel failed";
        return;
    }
    esp_wifi_get_mac(WIFI_IF_STA, localMac);
    if (esp_now_init() != ESP_OK) { initFault = "esp_now_init failed"; return; }
    if (esp_now_register_send_cb(gse_simple_send_shim) != ESP_OK ||
        esp_now_register_recv_cb(gse_simple_recv_shim) != ESP_OK) {
        initFault = "ESP-NOW callback registration failed";
        return;
    }
    started = true;
    initFault = "";

    const uint8_t padMac[MAC_LEN] = GSE_NOW_PAD_MAC;
    const uint8_t txMac[MAC_LEN]  = GSE_NOW_TX_MAC;
    memcpy(peerMac, GSE_LINK_IS_PAD ? txMac : padMac, MAC_LEN);
    peerConfigured = !isPlaceholder(peerMac);
    if (!peerConfigured) return;

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof peer);
    memcpy(peer.peer_addr, peerMac, MAC_LEN);
    peer.channel = (uint8_t)GSE_NOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ready = (esp_now_add_peer(&peer) == ESP_OK);
}

inline size_t GseLink::transmit(const uint8_t *data, size_t len)
{
    if (!ready || !data || len == 0 || len > ESP_NOW_MAX_DATA_LEN) return 0;
    return esp_now_send(peerMac, data, len) == ESP_OK ? len : 0;
}

inline void GseLink::push(const uint8_t *data, size_t len)
{
    portENTER_CRITICAL(&s_rxMux);
    size_t used = (uint8_t)(rxTail - rxHead);
    if (len > RX_CAP - 1 - used) {
        rxDropped = rxDropped + len;               /* drop whole, never split */
    } else {
        uint8_t t = rxTail;
        for (size_t i = 0; i < len; ++i) { rxBuf[t] = data[i]; t = (uint8_t)(t + 1); }
        rxTail = t;
    }
    portEXIT_CRITICAL(&s_rxMux);
}

inline void GseLink::onDatagram(const uint8_t *src, const uint8_t *data, int len)
{
    if (!ready || !src || !data || len <= 0) return;
    if (memcmp(src, peerMac, MAC_LEN) != 0) { rxForeign = rxForeign + 1; return; }

    if (len == (int)sizeof(gse_simple::RockPacket)) {
        gse_simple::RockPacket p;
        memcpy(&p, data, sizeof p);
        if (p.magic == gse_simple::kMagic && p.version == gse_simple::kVersion) {
            if (!GSE_LINK_IS_PAD && p.type == gse_simple::kTypePong) {
                pongSeq = p.sequence; pongSentAt = p.pingSentAtMs; pongArrived = true;
            } else if (GSE_LINK_IS_PAD && p.type == gse_simple::kTypePing) {
                pingInSeq = p.sequence; pingInSentAt = p.pingSentAtMs; pingArrived = true;
            }
            return;
        }
    }
    push(data, (size_t)len);                       /* a GSE frame */
}

inline void GseLink::sendRock(uint8_t type, uint32_t seq, uint32_t sentAt)
{
    gse_simple::RockPacket p;
    memset(&p, 0, sizeof p);
    p.magic = gse_simple::kMagic;
    p.version = gse_simple::kVersion;
    p.type = type;
    p.sequence = seq;
    p.pingSentAtMs = sentAt;
    transmit(reinterpret_cast<const uint8_t *>(&p), sizeof p);
}

inline void GseLink::poll()
{
    if (!ready) return;
    const uint32_t now = millis();
#if GSE_LINK_IS_PAD
    if (pingArrived) {                             /* answer the ground's PING */
        portENTER_CRITICAL(&s_rxMux);
        uint32_t seq = pingInSeq, sentAt = pingInSentAt;
        pingArrived = false;
        portEXIT_CRITICAL(&s_rxMux);
        sendRock(gse_simple::kTypePong, seq, sentAt);
    }
#else
    if (pongArrived) {
        portENTER_CRITICAL(&s_rxMux);
        uint32_t seq = pongSeq, sentAt = pongSentAt;
        pongArrived = false;
        portEXIT_CRITICAL(&s_rxMux);
        if (seq == pingSeq) pingAnswered = true;
        Serial.printf("LOG PONG seq=%lu rtt=%lu ms\n", (unsigned long)seq,
                      (unsigned long)(now - sentAt));
    }
    if (now - lastPingAt >= gse_simple::kPingIntervalMs) {
        if (!pingAnswered)
            Serial.printf("LOG PING seq=%lu: no PONG from the vehicle\n",
                          (unsigned long)pingSeq);
        lastPingAt = now;
        pingSeq++;
        pingAnswered = false;
        sendRock(gse_simple::kTypePing, pingSeq, now);
    }
#endif
}

/* ---------------------------------------------------------------- shims */
#if ESP_IDF_VERSION_MAJOR >= 5
static void gse_simple_recv_shim(const esp_now_recv_info_t *info,
                                 const uint8_t *data, int len)
{
    if (s_link && info) s_link->onDatagram(info->src_addr, data, len);
}
#else
static void gse_simple_recv_shim(const uint8_t *src, const uint8_t *data, int len)
{
    if (s_link) s_link->onDatagram(src, data, len);
}
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void gse_simple_send_shim(const esp_now_send_info_t *, esp_now_send_status_t status)
#else
static void gse_simple_send_shim(const uint8_t *, esp_now_send_status_t status)
#endif
{
    if (s_link) s_link->onSendStatus(status == ESP_NOW_SEND_SUCCESS);
}

#endif /* GSE_LINK_H */
