/*
 * link.h  --  the transport between the TX node and the pad node
 * ---------------------------------------------------------------------------
 * One object, one method set, two possible radios underneath:
 *
 *      GSE_LINK_LORA             DX-LR42/LR41-24T12x on UART1, 9600 8N1
 *      GSE_LINK_ESPNOW (default) ESP-NOW -- the ESP32's own Wi-Fi radio
 *
 * Everything above this header -- the 7E frames in gse_protocol.h, the CRC,
 * the sequence numbers, the pad's watchdog and arming rules, the TX node's
 * pacing and stale-frame rejection, the igniter interlock -- is identical in
 * both builds.  This file only moves bytes.  That is the point: a bug in the
 * safety logic cannot hide behind a change of radio, and the protocol tests
 * (tests/test_protocol.c) cover both because both carry the same frames.
 *
 * Why two radios
 *   * LoRa is the ECR configuration: hundreds of metres, its own radio, no
 *     Wi-Fi, no pairing.  It is optional, and the only link the original
 *     test plan signs off for flight.
 *   * ESP-NOW exists so the pair works with NO radio module at all: no UART
 *     wiring, no AT provisioning, no AUX pin, nothing to buy.  Two ESP32s and
 *     a laptop is enough for a bench run, a spare pad, or a short/medium-range
 *     link when the modules are lost or delayed.
 *
 * Why ESP-NOW rather than Bluetooth
 *   Bluetooth (BLE) was considered for a no-module link,
 *   but it is the wrong shape for this traffic:
 *     - it is connection-oriented, so there is pairing/bonding/GATT and a
 *       connected/disconnected state machine to reason about at power-up;
 *     - the default ATT payload is 20 bytes, so a 39-byte frame had to be
 *       chunked and reassembled;
 *     - "is the far end hearing me" is inferred, not measured.
 *   ESP-NOW is connectionless: a datagram addressed to the peer's MAC address,
 *   no association, no SSID, no router, one 250-byte packet, a per-packet
 *   delivery status from the link layer, and CCMP encryption keyed from
 *   compile-time constants instead of a pairing step.  A link failure is just
 *   "packets stop", which the pad's watchdog must handle. Range and
 *   interference still require hardware validation.
 *
 * Selecting it
 *   ESP-NOW is the ESP32 default. Use -DGSE_LINK_LORA on BOTH nodes to
 *   select the optional DX-LR42/LR41 module.  config.h enforces "exactly one of
 *   the two" and refuses ESP-NOW on the STM32 build, which has no Wi-Fi.
 *
 *   The peer's MAC address is compiled in (config.h, GSE_NOW_PAD_MAC /
 *   GSE_NOW_TX_MAC): a pad that accepted "whoever spoke first" would be a pad
 *   a stranger's sender could command.  First flash with the placeholders
 *   prints each board's own MAC; type the two MACs in, rebuild both, and the
 *   link comes up.
 *
 * A LoRa pad cannot hear an ESP-NOW TX node and vice versa.  That is not a
 * fault mode you should have to deduce: each node prints its transport on its
 * first log line ("PAD link: espnow ..."), and a mismatched pair shows
 * NO-LINK for ever while both boards look healthy.
 *
 * Method set (the seam)
 *   begin()                start the radio / the Wi-Fi stack + ESP-NOW
 *   poll()                 per-loop housekeeping; empty for both transports
 *                          (ESP-NOW delivers through its own callbacks)
 *   describe()             one line for the boot banner: what this link is
 *   provision(...)         LoRa: write and verify the AT parameters.
 *                          ESP-NOW: apply the channel and register the peer;
 *                          returns "" on success and the reason on failure, so
 *                          the callers' RADIO OK / RADIO FAULT logic is one
 *                          piece of code.
 *   send(dst, ch, frame, n)  frame is a bare 7E frame.  LoRa prefixes the
 *                          fixed-point destination address + channel; ESP-NOW
 *                          addresses by MAC and sends it unchanged.
 *   sendRaw(pkt, n)        pkt already carries the LoRa 3-byte prefix (the
 *                          portable TX core builds it, see gse_tx_core.c).
 *                          LoRa writes it untouched; ESP-NOW strips it, because
 *                          those three bytes are not addressing here, they are
 *                          noise the parser would have to resync past.
 *   read() / available()   the same polled byte stream in both builds.
 *   linkUp()               LoRa: always true (the module is a dumb pipe).
 *                          ESP-NOW: radio initialised, peer MAC configured and
 *                          registered.  Whether the far end is alive is the
 *                          protocol's business (HELLO/STATE + the watchdog).
 */
#ifndef GSE_LINK_H
#define GSE_LINK_H

#include <Arduino.h>

#include "config.h"
#include "gse_protocol.h"

#if defined(GSE_ROLE_PAD)
#  define GSE_LINK_IS_PAD 1
#else
#  define GSE_LINK_IS_PAD 0
#endif

#if defined(GSE_LINK_ESPNOW)

/* ========================================================================== *
 *  ESP-NOW transport
 * ========================================================================== *
 *
 * API compatibility.  This matters, because the IDF API changed under us and
 * this repo supports both Arduino cores:
 *
 *   arduino-esp32 2.x  ->  ESP-IDF 4.4
 *       typedef void (*esp_now_recv_cb_t)(const uint8_t *mac_addr,
 *                                         const uint8_t *data, int data_len);
 *       RSSI lives in wifi_pkt_rx_ctrl_t and is not passed to the callback.
 *
 *   arduino-esp32 3.x  ->  ESP-IDF 5.x
 *       typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t *info,
 *                                         const uint8_t *data, int data_len);
 *       RSSI is info->rx_ctrl->rssi.
 *
 * Everything else used here -- esp_now_init, esp_now_add_peer, esp_now_send,
 * esp_now_register_recv_cb, esp_now_register_send_cb, esp_now_set_pmk,
 * esp_now_peer_info_t, ESP_NOW_MAX_DATA_LEN, ESP_ERR_ESPNOW_EXIST,
 * ESP_NOW_SEND_SUCCESS -- is shared between 4.4 and 5.x. The receive callback
 * changed in IDF 5.0; the send callback changed in 5.5 from a MAC pointer to
 * an esp_now_send_info_t pointer. Both callbacks are shimmed to the signature
 * declared by the installed IDF before passing data to the project code.
 */

#include <esp_err.h>
#include <esp_event.h>
#include <esp_idf_version.h>
#include <esp_netif.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <stdio.h>
#include <string.h>

/* ESP-IDF's key types are fixed 16-byte arrays.  A silently truncated key
 * would fail at run time as "send failed", which reads like a radio fault, so
 * the length is a build-time requirement instead. */
static_assert(sizeof(GSE_NOW_KEY_TEXT) - 1 == 16,
              "GSE_NOW_KEY_TEXT must be exactly 16 characters");
static_assert(sizeof(GSE_NOW_PMK_TEXT) - 1 == 16,
              "GSE_NOW_PMK_TEXT must be exactly 16 characters");

class GseLink;

/* One link per node.  The receive callback runs in the Wi-Fi task, so it can
 * only reach the object through a file-scope pointer; it is bound in begin(),
 * before esp_now_init(), so it is never null when the stack calls it. */
static GseLink *s_link = nullptr;

/* Guards the receive ring, written from the Wi-Fi task and read from loop().
 * Critical sections are a few instructions long (a byte at a time). */
static portMUX_TYPE s_rxMux = portMUX_INITIALIZER_UNLOCKED;

/* The received-packet shims.  Declared here, defined at the bottom of this
 * block with the signature the current IDF expects. */
#if ESP_IDF_VERSION_MAJOR >= 5
static void gse_now_recv_shim(const esp_now_recv_info_t *info,
                              const uint8_t *data, int len);
#else
static void gse_now_recv_shim(const uint8_t *src_mac, const uint8_t *data,
                              int len);
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void gse_now_send_shim(const esp_now_send_info_t *info, esp_now_send_status_t status);
#else
static void gse_now_send_shim(const uint8_t *mac, esp_now_send_status_t status);
#endif

class GseLink {
public:
    /* 256 bytes: about six of the largest (39 B) frames in flight, which is
     * more than one 20 ms control tick can produce in either direction, so the
     * overflow path below is a last resort rather than a design point.  Power
     * of two because the head/tail indices are uint8_t: they wrap by
     * themselves, need no modulo, and a single-byte write inside the critical
     * section is atomic. */
    enum { RX_CAP = 256 };
    enum { MAC_LEN = 6 };

    GseLink()
        : rxHead(0), rxTail(0), dropped(0),
          rxDatagrams(0), txDatagrams(0), txOk(0), txFailed(0), rxForeign(0),
          lastRxMs(0), lastTxOkMs(0), lastRssi(0),
          started(false), peerRegistered(false), ready(false),
          peerConfigured(false), initFault("not started")
    {
        memset(peerMac, 0, sizeof peerMac);
        memset(localMac, 0, sizeof localMac);
    }

    /* ------------------------------------------------------------ lifecycle */
    void begin();
    void poll() {}                    /* ESP-NOW pushes; nothing to poll    */

    const char *transport() const { return "espnow"; }

    String describe() const;

    /* See the header comment.  begin() has already done the work; this only
     * reports whether it succeeded and why not. */
    String provision(uint8_t, uint8_t, uint16_t, uint16_t, uint16_t);

    /* -------------------------------------------------------------- framing */
    /* frame = bare 7E frame; the LoRa addressing arguments are unused here. */
    size_t send(uint16_t, uint8_t, const uint8_t *frame, size_t len)
    {
        return transmit(frame, len);
    }

    /* pkt = [addr_hi addr_lo chan] + 7E frame, as the portable TX core builds
     * it for the module's fixed-point mode.  Strip those three bytes. */
    size_t sendRaw(const uint8_t *pkt, size_t len)
    {
        if (len <= 3) return 0;
        return transmit(pkt + 3, len - 3);
    }

    int available() const
    {
        portENTER_CRITICAL(&s_rxMux);
        int n = (int)((rxTail + RX_CAP - rxHead) % RX_CAP);
        portEXIT_CRITICAL(&s_rxMux);
        return n;
    }

    int read()
    {
        portENTER_CRITICAL(&s_rxMux);
        int b = -1;
        if (rxHead != rxTail) {
            b = rxBuf[rxHead];
            rxHead = (uint8_t)((rxHead + 1) % RX_CAP);
        }
        portEXIT_CRITICAL(&s_rxMux);
        return b;
    }

    bool linkUp() const { return ready; }

    const char *linkState() const
    {
        if (!started)        return "radio failed";
        if (!peerConfigured) return "peer MAC NOT CONFIGURED";
        if (!peerRegistered) return "peer registration failed";
        return "ready (connectionless)";
    }

    /* Real link-layer numbers for the nodes' 1 Hz line.  "acks" counts
     * ESP_NOW_SEND_SUCCESS from the radio, which is the only honest answer to
     * "is the other board hearing me" -- the protocol's own counters say what
     * it did with what arrived. */
    unsigned long ackCount() const { return txOk; }
    unsigned long failCount() const { return txFailed; }
    unsigned long foreignCount() const { return rxForeign; }
    unsigned long rxCount() const { return rxDatagrams; }
    int rssi() const { return (int)lastRssi; }
    bool heardRecently(uint32_t now, uint32_t withinMs) const
    {
        return lastRxMs && (now - lastRxMs) < withinMs;
    }

    /* ------------------------------------------------------------ callbacks
     * Called from the Wi-Fi task.  Copy bytes, record status, return. */
    void onDatagram(const uint8_t *src, const uint8_t *data, int len, int rssi);
    void onSendStatus(esp_now_send_status_t status);

    /* ---------------------------------------------------------------- state */
    uint8_t  rxBuf[RX_CAP];
    volatile uint8_t  rxHead, rxTail;
    volatile uint32_t dropped;         /* bytes lost to ring-buffer overflow */
    volatile uint32_t rxDatagrams, txDatagrams, txOk, txFailed, rxForeign;
    volatile uint32_t lastRxMs, lastTxOkMs;
    volatile int8_t   lastRssi;
    bool     started, peerRegistered, ready, peerConfigured;
    const char *initFault;
    uint8_t  peerMac[MAC_LEN];
    uint8_t  localMac[MAC_LEN];

private:
    void     push(const uint8_t *data, size_t len);
    size_t   transmit(const uint8_t *data, size_t len);
    bool     addPeer();
    static bool isPlaceholder(const uint8_t *mac);
    static String macToStr(const uint8_t *m);

    /* rxHead/rxTail are uint8_t and updated inside a critical section: single
     * byte writes are atomic on both ESP32 cores, and that only holds while the
     * capacity fits.  Assert it rather than corrupt a ring buffer. */
    static_assert(RX_CAP <= 256 && (RX_CAP & (RX_CAP - 1)) == 0,
                  "RX_CAP must be a power of two <= 256 for the uint8_t indices");
};

/* -------------------------------------------------------------------- code */

inline String GseLink::macToStr(const uint8_t *m)
{
    char b[18];
    snprintf(b, sizeof b, "%02X:%02X:%02X:%02X:%02X:%02X",
             m[0], m[1], m[2], m[3], m[4], m[5]);
    return String(b);
}

inline String GseLink::describe() const
{
    if (!peerConfigured) {
        /* The most likely state on a first flash, so say the two things the
         * builder needs: this board's own MAC, and what is missing. */
        String s = String("ESP-NOW ch ") + String((int)GSE_NOW_CHANNEL) +
                   ", local MAC " + macToStr(localMac) +
                   ", peer MAC NOT CONFIGURED (set GSE_NOW_" +
                   (GSE_LINK_IS_PAD ? "TX" : "PAD") + "_MAC and rebuild)";
        return s;
    }
    return String("ESP-NOW ch ") + String((int)GSE_NOW_CHANNEL) +
           ", local " + macToStr(localMac) + " -> peer " + macToStr(peerMac) +
           ", CCMP on, no Wi-Fi association";
}

inline String GseLink::provision(uint8_t, uint8_t, uint16_t, uint16_t, uint16_t)
{
    if (!started)
        return String(initFault);
    if (!peerConfigured)
        return String("peer MAC not configured: flash once, read this node's "
                      "local MAC from the banner above, put both MACs in "
                      "config.h (GSE_NOW_PAD_MAC / GSE_NOW_TX_MAC), rebuild");
    if (!peerRegistered) return String("esp_now_add_peer failed");
    return String();
}

inline bool GseLink::isPlaceholder(const uint8_t *mac)
{
    /* The documented placeholders are 02:00:00:00:00:01 (TX) and
     * 02:00:00:00:00:02 (pad) -- locally administered, obviously fake.  A real
     * station MAC carries the Espressif OUI and never looks like this. */
    return mac[0] == 0x02 && mac[1] == 0x00 && mac[2] == 0x00 &&
           mac[3] == 0x00 && mac[4] == 0x00 &&
           (mac[5] == 0x01 || mac[5] == 0x02);
}

inline void GseLink::push(const uint8_t *data, size_t len)
{
    portENTER_CRITICAL(&s_rxMux);
    uint8_t tail = rxTail;
    size_t used = (rxTail + RX_CAP - rxHead) % RX_CAP;
    if (len > RX_CAP - 1 - used) {
        /* Drop a whole datagram, never overwrite a partially consumed frame. */
        dropped = dropped + (uint32_t)len;
    } else {
        for (size_t i = 0; i < len; ++i) {
            rxBuf[tail] = data[i];
            tail = (uint8_t)(tail + 1);
        }
        rxTail = tail;
    }
    portEXIT_CRITICAL(&s_rxMux);
}

inline void GseLink::onDatagram(const uint8_t *src, const uint8_t *data,
                                int len, int rssi)
{
    if (!data || len <= 0 || !peerRegistered) return;

    /* Only the configured peer.  With CCMP on *and* the peer MAC pinned in
     * config.h this is belt-and-braces, but it is also what turns "some other
     * ESP-NOW device in the area" from a protocol-level puzzle into a counter
     * the log can show. */
    if (!src || memcmp(src, peerMac, MAC_LEN) != 0) {
        rxForeign = rxForeign + 1;
        return;
    }

    lastRxMs = millis();
    lastRssi = (int8_t)rssi;
    rxDatagrams = rxDatagrams + 1;
    push(data, (size_t)len);
}

inline void GseLink::onSendStatus(esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        txOk = txOk + 1;
        lastTxOkMs = millis();
    } else {
        txFailed = txFailed + 1;
    }
}

inline bool GseLink::addPeer()
{
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof peer);
    memcpy(peer.peer_addr, peerMac, MAC_LEN);
    peer.channel = (uint8_t)GSE_NOW_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = true;                       /* CCMP, key below */
    /* ESP-IDF takes the peer's LMK (local master key) per peer; the PMK is set
     * once, globally, in begin().  Both ends must use the same pair or CCMP
     * decryption fails -- and ESP-NOW reports a decrypt failure as a *failed
     * send*, not as a decrypt error, so a key mismatch looks like "the TX node
     * says 'send failed' and the pad hears nothing".  That symptom is called
     * out in docs/LINK_OPTIONS.md. */
    memcpy(peer.lmk, GSE_NOW_KEY_TEXT, 16);

    esp_err_t err = esp_now_add_peer(&peer);
    /* An existing peer may have different encryption parameters; do not
     * silently claim it is safe or ready. */
    return err == ESP_OK;
}

inline void GseLink::begin()
{
    s_link = this;                     /* before init: the stack may fire   */

    /* Fail closed if any radio setup step fails. Do not associate with Wi-Fi. */
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { initFault = "esp_netif_init failed"; return; }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { initFault = "event loop init failed"; return; }
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&wcfg) != ESP_OK) { initFault = "esp_wifi_init failed"; return; }
    if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) { initFault = "Wi-Fi storage failed"; return; }
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) { initFault = "Wi-Fi STA mode failed"; return; }
    if (esp_wifi_start() != ESP_OK) { initFault = "esp_wifi_start failed"; return; }
    if (esp_wifi_get_mac(WIFI_IF_STA, localMac) != ESP_OK) { initFault = "station MAC read failed"; return; }
    if (esp_wifi_set_channel((uint8_t)GSE_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        initFault = "Wi-Fi channel failed"; return;
    }
    if (esp_now_init() != ESP_OK) { initFault = "esp_now_init failed"; return; }
    if (esp_now_set_pmk((const uint8_t *)GSE_NOW_PMK_TEXT) != ESP_OK) {
        initFault = "ESP-NOW PMK failed"; return;
    }
    if (esp_now_register_recv_cb(gse_now_recv_shim) != ESP_OK) {
        initFault = "ESP-NOW receive callback failed"; return;
    }
    if (esp_now_register_send_cb(gse_now_send_shim) != ESP_OK) {
        initFault = "ESP-NOW send callback failed"; return;
    }
    started = true;
    initFault = "";

    /* 4. The peer, from config.h. */
    const uint8_t padMac[MAC_LEN] = GSE_NOW_PAD_MAC;
    const uint8_t txMac[MAC_LEN]  = GSE_NOW_TX_MAC;
    const uint8_t *want = GSE_LINK_IS_PAD ? txMac : padMac;
    memcpy(peerMac, want, MAC_LEN);
    peerConfigured = !isPlaceholder(peerMac);

    if (!peerConfigured) {
        /* Not ready, and said so: this node must NOT report LINK OK while it
         * cannot possibly receive.  describe() prints the local MAC to type
         * into the other board's config.h. */
        peerRegistered = false;
        ready = false;
        return;
    }

    peerRegistered = addPeer();
    ready = peerRegistered;
}

inline size_t GseLink::transmit(const uint8_t *data, size_t len)
{
    if (!ready || !data) return 0;

    /* ESP-NOW carries 250 bytes in one datagram and the largest frame this
     * protocol can produce is 39, so -- unlike the Bluetooth transport this
     * replaces -- there is no chunking, no reassembly, and no partial frame to
     * reason about.  Assert the relationship at compile time instead of
     * discovering it as a truncated command. */
    static_assert(GSE_MAX_FRAME + 3 <= ESP_NOW_MAX_DATA_LEN,
                  "a GSE frame must fit one ESP-NOW datagram");
    if (len > ESP_NOW_MAX_DATA_LEN) return 0;

    if (esp_now_send(peerMac, data, len) != ESP_OK) {
        txFailed = txFailed + 1;
        return 0;
    }
    txDatagrams = txDatagrams + 1;
    return len;
}

/* ------------------------------------------------------------------- shims
 * The version-specific callbacks for this transport. Note the RSSI source:
 * IDF 5 hands it over in the packet info, IDF 4 does not pass it at all (it
 * lives in the rx control block of the promiscuous/RX path), so the 4.x shim
 * reports 0 -- meaning "unknown", not "zero dBm", and the log says so. */
#if ESP_IDF_VERSION_MAJOR >= 5
static void gse_now_recv_shim(const esp_now_recv_info_t *info,
                              const uint8_t *data, int len)
{
    if (!s_link) return;
    const uint8_t *src = info ? info->src_addr : nullptr;
    int rssi = (info && info->rx_ctrl) ? (int)info->rx_ctrl->rssi : 0;
    s_link->onDatagram(src, data, len, rssi);
}
#else
static void gse_now_recv_shim(const uint8_t *src_mac, const uint8_t *data,
                              int len)
{
    if (!s_link) return;
    s_link->onDatagram(src_mac, data, len, 0);
}
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void gse_now_send_shim(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    (void)info;
#else
static void gse_now_send_shim(const uint8_t *mac, esp_now_send_status_t status)
{
    (void)mac;
#endif
    if (s_link) s_link->onSendStatus(status);
}

#else  /* ------------------------------------------------------- GSE_LINK_LORA
        *
        * The optional transport: the DX-LR42/LR41 module on UART1.  This is the
        * code that used to sit in pad_node.cpp and tx_platform_esp32.cpp --
        * delegated to the same driver, not rewritten, so the provisioning and
        * the wire format are byte-for-byte what the test plan signs off.
        */

#include "dxlr41.h"

class GseLink {
public:
    void begin() { radio_.begin(DXLR41_BAUD); }
    void poll()  {}

    const char *transport() const { return "lora"; }

    String describe() const
    {
        String s = String("LoRa UART1 RX<-D") + String(PIN_LORA_RX) +
                   " TX->D" + String(PIN_LORA_TX) + " AUX=";
        s += radio_.auxEnabled() ? "wired" : "NOT WIRED (pacing only)";
        s += " M0/M1=";
        s += PIN_LORA_M0 >= 0 ? "driven low" : "tie to GND at the module";
        return s;
    }

    String provision(uint8_t level, uint8_t ch, uint16_t key,
                     uint16_t my, uint16_t peer)
    {
        return radio_.configure(level, ch, key, true, my, peer);
    }

    /* Bare frame in; the module's fixed-point mode wants the destination
     * address (2 B) + destination channel (1 B) in front of it. */
    size_t send(uint16_t dst, uint8_t chan, const uint8_t *frame, size_t len)
    {
        uint8_t pkt[GSE_MAX_FRAME + 4];
        size_t m = radio_.buildFixedPoint(pkt, dst, chan, frame, len);
        return radio_.write(pkt, m);
    }

    /* The caller already prefixed it (the portable TX core does). */
    size_t sendRaw(const uint8_t *pkt, size_t len) { return radio_.write(pkt, len); }

    int available() { return radio_.available(); }
    int read()      { return radio_.read(); }

    bool linkUp() const { return true; }
    const char *linkState() const { return "module pipe"; }

    /* The module's own AT+HELP dump (plus the AT+SWITCH answer, appended by
     * the driver), for the console. */
    const char *banner() const { return radio_.banner(); }
    bool auxEnabled() const { return radio_.auxEnabled(); }
    /* 0 = modes by AT command, 1 = modes by M0/M1 pins, -1 = no answer. */
    int switchMode() const { return radio_.switchMode(); }

private:
    DxLr41 radio_{Serial1, PIN_LORA_AUX, PIN_LORA_M0, PIN_LORA_M1,
                  PIN_LORA_AUX_PULLUP != 0};
};

#endif /* GSE_LINK_ESPNOW */

#endif /* GSE_LINK_H */
