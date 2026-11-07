/*
 * link.h  --  the transport between the TX node and the pad node
 * ---------------------------------------------------------------------------
 * One object, one method set, two possible radios underneath:
 *
 *      GSE_LINK_LORA (default)   DX-LR42/LR41-24T12x on UART1, 9600 8N1
 *      GSE_LINK_BLE              the ESP32's own Bluetooth Low Energy radio
 *
 * Everything above this header -- the 7E frames in gse_protocol.h, the CRC,
 * the sequence numbers, the pad's watchdog and arming rules, the TX node's
 * pacing and stale-frame rejection, the igniter interlock -- is identical in
 * both builds.  This file only moves bytes.  That is the point: a bug in the
 * safety logic cannot hide behind a change of radio, and the protocol tests
 * (tests/test_protocol.c) cover both because both carry the same frames.
 *
 * Why two radios
 *   * LoRa is the ECR configuration: hundreds of metres, no pairing, no
 *     phones, works with Bluetooth and Wi-Fi off.  It is the default, and the
 *     only link the current test plan signs off for flight.
 *   * BLE exists so the pair works with NO radio module at all: no UART
 *     wiring, no AT provisioning, no AUX pin, nothing to buy.  Two ESP32s and
 *     a laptop is enough for a bench run, a spare pad, or a short-range link
 *     when the modules are lost or delayed.  Its range is a fraction of the
 *     LoRa link's -- see docs/LINK_OPTIONS.md, which is the operator-facing
 *     comparison, and the security note in config.h.
 *
 * Selecting it
 *   -DGSE_LINK_BLE on BOTH nodes (platformio.ini envs pad-node-ble /
 *   tx-node-ble and their -esp32s twins).  config.h enforces "exactly one of
 *   the two", and refuses BLE on the STM32 build, which has no Bluetooth.
 *
 * A LoRa pad cannot hear a BLE TX node and vice versa.  That is not a fault
 * mode you should have to deduce: each node prints its transport in the boot
 * banner ("PAD link: ble ..."), and a mismatched pair shows NO-LINK forever.
 *
 * Method set (the seam)
 *   begin()                start the radio / the Bluetooth stack
 *   poll()                 per-loop housekeeping (BLE scan + connect); no-op
 *                          on LoRa.  Must never block.
 *   describe()             one line for the boot banner: what this link is
 *   provision(...)         LoRa: write and verify the AT parameters.
 *                          BLE: nothing to provision -- the link parameters
 *                          are compile-time constants in config.h -- so it
 *                          only reports whether the stack started.  Returns
 *                          an empty String for "no fault", exactly like LoRa,
 *                          so the callers' RADIO OK / RADIO FAULT logic is
 *                          one piece of code.
 *   send(dst, ch, frame, n)  frame is a bare 7E frame.  LoRa prefixes the
 *                          fixed-point destination address + channel; BLE has
 *                          its own addressing and sends it unchanged.
 *   sendRaw(pkt, n)        pkt already carries the LoRa 3-byte prefix (the
 *                          portable TX core builds it, see gse_tx_core.c).
 *                          LoRa writes it untouched; BLE strips it, because
 *                          over BLE those three bytes are not addressing, they
 *                          are noise the parser would have to resync past.
 *   read() / available()   the same polled byte stream in both builds.
 *   linkUp()               LoRa: always true (the module is a dumb pipe;
 *                          whether the far end is alive is the protocol's
 *                          business, and the watchdog already covers it).
 *                          BLE: an actual GATT connection exists.
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

#if defined(GSE_LINK_BLE)

/* ------------------------------------------------------------------------- *
 *  Bluetooth Low Energy transport
 * ------------------------------------------------------------------------- *
 *
 * Topology: the PAD is the peripheral (GATT server, advertises GSE_BLE_NAME),
 * the TX node is the central (GATT client, scans for that service).  Chosen
 * deliberately:
 *
 *   * the pad never initiates anything, so nothing that goes wrong on the TX
 *     side can stall the pad's 20 ms control loop;
 *   * a phone cannot become the pad's peer by accident -- the pad does not
 *     advertise to scanners that look only for names, and it ignores writes
 *     from a central that is not subscribed to the right characteristics.
 *     There is no pairing, so this is "not easy to stumble into", not
 *     "impossible": see the security note in config.h;
 *   * the TX node, which has a laptop attached and a human watching it, is
 *     where "it is scanning, nothing found" is a useful message.
 *
 * Only the BLE API common to arduino-esp32 2.x and 3.x is used (this repo
 * supports both cores; tests/esp32_stub/ stands in for them on a host build).
 * The three API differences that actually matter here, and how each is
 * avoided:
 *
 *   * BLEScan::start() returns BLEScanResults by value in 2.x and by pointer
 *     in 3.x -- the scan is driven by callbacks and stop() instead, and the
 *     return value is never touched;
 *   * BLEAdvertisedDevice::getName() returns std::string in 2.x and String in
 *     3.x -- devices are matched on the advertised service UUID, with a name
 *     check done through .c_str(), which is the same call in both;
 *   * BLEDevice::init() returns void in 2.x and bool in 3.x -- called as a
 *     statement, value ignored.
 *
 * Payload size: a GSE frame is up to GSE_MAX_FRAME (39) bytes and the default
 * ATT MTU allows 20 bytes per write/notification, so every frame goes out as
 * two chunks and is reassembled by the peer's byte-stream parser.  MTU
 * negotiation is deliberately not attempted: 40 packets/s at 20 Hz is nothing
 * for BLE, and it is one fewer thing that can differ between core versions.  A
 * lost chunk costs exactly one frame (20 ms); the parser resyncs on the next
 * 0x7E with a valid version byte, so half a frame can never be mistaken for a
 * command, and the pad's watchdog does not even notice a single 20 ms gap.
 *
 * Threading: all four callbacks run in the Bluetooth task, not in loop().  They
 * only copy bytes into a critical-section-protected ring buffer, set a flag or
 * record a MAC address.  No parsing, no logging, no allocation in there.
 */

#include <BLEDevice.h>
#include <BLE2902.h>
#include <BLEUtils.h>

class GseLink;

/* One link per node, so a file-scope pointer is the right amount of machinery.
 * The Bluetooth task reaches the object through it; it is bound in begin(),
 * before the stack can fire a callback.  `static` because exactly one
 * translation unit per firmware image includes this header (pad_node.cpp or
 * tx_platform_esp32.cpp) -- see the note in tests/esp32_stub/BLE*.h. */
static GseLink *gse_ble_link = nullptr;

/* Guards the receive ring buffer, which is written from the Bluetooth task and
 * read from loop().  A byte at a time in and out: the critical sections are a
 * few instructions long. */
static portMUX_TYPE s_rxMux = portMUX_INITIALIZER_UNLOCKED;

/* ---------------------------------------------------------------- callbacks
 * One object implements every BLE callback interface the node needs, so the
 * stack holds one pointer and the hand-off back into GseLink is one line each.
 * The one-argument onConnect/onDisconnect/onWrite forms are the ones common to
 * 2.x and 3.x; the overloads that take an event struct are left alone. */
class GseBleCallbacks : public BLEServerCallbacks,
                        public BLECharacteristicCallbacks,
                        public BLEAdvertisedDeviceCallbacks,
                        public BLEClientCallbacks {
public:
    void onConnect(BLEServer *srv) override;
    void onDisconnect(BLEServer *srv) override;
    void onWrite(BLECharacteristic *chr) override;
    void onResult(BLEAdvertisedDevice dev) override;
    void onConnect(BLEClient *cli) override;
    void onDisconnect(BLEClient *cli) override;

    /* Scan-complete, in the two signatures the two cores use.  Both are
     * routed to one hook, because the scan object does not survive a stop():
     * until the application re-registers its callback on the (fresh) scan
     * object, results are silently dropped -- which reads as "it scanned and
     * found nothing" even with the pad powered and two metres away. */
    void scanCompleted();

    /* Called by GseLink once a connection is up: the TX node has no interest
     * in further advertisements, and the pad must not be reconnected to by a
     * second central. */
    volatile bool active;
    GseBleCallbacks() : active(false) {}
};

class GseLink {
public:
    /* 256 bytes = ~6 of the largest (39 B) frames in flight, and the reason
     * it is a power of two is that the head/tail indices are uint8_t: they
     * wrap by themselves, so the ring needs no modulo and a byte write inside
     * the critical section is atomic.  255 usable bytes. */
    enum { RX_CAP = 256 };

    GseLink()
        : rxHead(0), rxTail(0), dropped(0), connected(false), scanning(false),
          started(false), targetSeen(false), scanStartMs(0), lastAttemptMs(0),
          targetType(0)
#if GSE_LINK_IS_PAD
          , server(nullptr), rxChar(nullptr), txChar(nullptr)
#else
          , client(nullptr), rxRemote(nullptr)
#endif
    { targetMac[0] = 0; }

    /* ------------------------------------------------------------ lifecycle */
    void begin();
    void poll();

    const char *transport() const { return "ble"; }

    String describe() const
    {
#if GSE_LINK_IS_PAD
        return String("BLE peripheral \"") + GSE_BLE_NAME +
               "\" (svc ff00, rx ff01, tx ff02), no module, no pairing";
#else
        return String("BLE central, scanning for service ff00 for \"") +
               GSE_BLE_NAME + "\", no module, no pairing";
#endif
    }

    /* See the header comment: nothing to write, only "did the stack start".
     * Also doubles as the "reconnect now" button behind RADIO CONFIG. */
    String provision(uint8_t, uint8_t, uint16_t, uint16_t, uint16_t)
    {
        if (!started) return String("BLE stack not started");
#if !GSE_LINK_IS_PAD
        restartScan();
#endif
        return String();
    }

    /* -------------------------------------------------------------- framing */
    /* frame = bare 7E frame; the LoRa addressing arguments are unused here. */
    size_t send(uint16_t, uint8_t, const uint8_t *frame, size_t len)
    {
        return writeChunks(frame, len);
    }

    /* pkt = [addr_hi addr_lo chan] + 7E frame (the module's fixed-point form). */
    size_t sendRaw(const uint8_t *pkt, size_t len)
    {
        if (len <= 3) return 0;
        return writeChunks(pkt + 3, len - 3);
    }

    int available() const
    {
        return (int)((rxTail + RX_CAP - rxHead) % RX_CAP);
    }

    int read()
    {
        if (rxHead == rxTail) return -1;
        uint8_t b = rxBuf[rxHead];
        rxHead = (uint8_t)((rxHead + 1) % RX_CAP);
        return (int)b;
    }

    bool linkUp() const { return connected; }

    const char *linkState() const
    {
        if (connected) return "connected";
#if GSE_LINK_IS_PAD
        return "advertising (no TX node connected)";
#else
        return scanning ? "scanning for the pad" : "not connected";
#endif
    }

    /* ------------------------------------------------------- callback hooks */
    void pushFromBle(const uint8_t *data, size_t len);
    void setConnected(bool up) { connected = up; }
    void noteTarget(const char *mac, uint8_t type);
    /* The stack says the scan window expired without a match: stop waiting and
     * start the back-off timer, so the next scan is issued from loop() (where
     * it is safe to allocate) rather than from the scan callback. */
    void scanFinished();
    /* The scan stops on *finding* the pad as well, and the Arduino BLE layer
     * does not re-register the results callback after a stop: re-arm it before
     * every start, or the second scan finds nothing. */
    void armScanCallbacks();

    /* ---------------------------------------------------------------- state */
    uint8_t  rxBuf[RX_CAP];
    volatile uint8_t  rxHead, rxTail;
    volatile uint32_t dropped;        /* bytes lost to ring-buffer overflow  */
    volatile bool connected;
    volatile bool scanning;
    bool     started;
    volatile bool targetSeen;
    volatile uint32_t scanStartMs;
    volatile uint32_t lastAttemptMs;
    char     targetMac[18];
    uint8_t  targetType;

#if GSE_LINK_IS_PAD
    BLEServer *server;
    BLECharacteristic *rxChar;        /* central writes -> pad receives      */
    BLECharacteristic *txChar;        /* pad notifies    -> central receives */
#else
    BLEClient *client;
    BLERemoteCharacteristic *rxRemote;
#endif

private:
    /* rxHead/rxTail are uint8_t, because they are updated inside a critical
     * section and a single-byte write is atomic on both ESP32 cores.  That
     * only works while the capacity fits in a uint8_t -- assert it rather than
     * discover it as a corrupted ring buffer. */
    static_assert(RX_CAP <= 256 && (RX_CAP & (RX_CAP - 1)) == 0,
                  "RX_CAP must be a power of two <= 256 for the uint8_t indices");

    size_t writeChunks(const uint8_t *p, size_t len);
    void startAdvertising();
#if !GSE_LINK_IS_PAD
    void restartScan();
    bool connectAndSubscribe();
#endif
};

/* One callbacks object for the whole node: the stack keeps a pointer to it for
 * the life of the process, so it must have static storage duration. */
static GseBleCallbacks gse_ble_cb;

/* BLEScan::start() takes a plain function pointer for its completion callback.
 * arduino-esp32 2.0.x and 3.0.x both declare it as void(*)(BLEScanResults)
 * (3.1 changed it again); this trampoline matches that, so `nullptr` is not
 * needed and the overload is unambiguous in every supported core.  The results
 * are not inspected -- the pad's advertisement already arrived through
 * onResult() -- only the "the window closed, nothing matched" edge matters. */
static void gse_ble_scan_done(BLEScanResults) { gse_ble_cb.scanCompleted(); }

/* The receive trampoline for the TX node: the BLE client hands us packets,
 * we copy them into the same ring buffer the pad fills from GATT writes. */
inline void gse_ble_notify(BLERemoteCharacteristic *, uint8_t *data, size_t len,
                           bool)
{
    if (gse_ble_link) gse_ble_link->pushFromBle(data, len);
}

/* -------------------------------------------------------------------- code */

inline void GseBleCallbacks::onConnect(BLEServer *) { if (gse_ble_link) gse_ble_link->setConnected(true); }
inline void GseBleCallbacks::onDisconnect(BLEServer *) { if (gse_ble_link) gse_ble_link->setConnected(false); }
inline void GseBleCallbacks::onConnect(BLEClient *) { if (gse_ble_link) gse_ble_link->setConnected(true); }
inline void GseBleCallbacks::onDisconnect(BLEClient *) { if (gse_ble_link) gse_ble_link->setConnected(false); }

inline void GseBleCallbacks::onWrite(BLECharacteristic *chr)
{
    if (!gse_ble_link || !chr) return;
    gse_ble_link->pushFromBle(chr->getData(), chr->getLength());
}

inline void GseBleCallbacks::scanCompleted()
{
    if (gse_ble_link && !gse_ble_link->targetSeen) gse_ble_link->scanFinished();
}

inline void GseBleCallbacks::onResult(BLEAdvertisedDevice dev)
{
    if (!gse_ble_link) return;
    /* Match on the service UUID we advertise -- reliable in both cores, and
     * a stronger filter than a name that anyone could copy.  The name check
     * is a fallback for a central whose scan response was truncated. */
    bool match = dev.isAdvertisingService(BLEUUID((uint16_t)GSE_BLE_SVC_UUID));
    if (!match && dev.haveName())
        match = strcmp(dev.getName().c_str(), GSE_BLE_NAME) == 0;
    if (!match) return;

    BLEAddress a = dev.getAddress();
    gse_ble_link->noteTarget(a.toString().c_str(), dev.getAddressType());
}

inline void GseLink::scanFinished()
{
    scanning = false;
    lastAttemptMs = millis();
}

inline void GseLink::armScanCallbacks()
{
    BLEScan *scan = BLEDevice::getScan();
    scan->setActiveScan(true);
    scan->setAdvertisedDeviceCallbacks(&gse_ble_cb, false, true);
}

inline void GseLink::noteTarget(const char *mac, uint8_t type)
{
    if (targetSeen || !mac) return;
    size_t i = 0;
    for (; mac[i] && i + 1 < sizeof(targetMac); ++i) targetMac[i] = mac[i];
    targetMac[i] = 0;
    targetType = type;
    targetSeen = true;
}

inline void GseLink::pushFromBle(const uint8_t *data, size_t len)
{
    if (!data) return;
    uint8_t tail = rxTail;
    portENTER_CRITICAL(&s_rxMux);
    for (size_t i = 0; i < len; ++i) {
        uint8_t next = (uint8_t)((tail + 1) % RX_CAP);
        if (next == rxHead) {                 /* full: drop the oldest byte */
            rxHead = (uint8_t)((rxHead + 1) % RX_CAP);
            dropped++;
        }
        rxBuf[tail] = data[i];
        tail = next;
    }
    rxTail = tail;
    portEXIT_CRITICAL(&s_rxMux);
}

inline void GseLink::begin()
{
    gse_ble_link = this;
    /* ~1 s on the first boot (controller + host task).  Called before the
     * pad's watchdog is armed and while the outputs are still held safe. */
    BLEDevice::init(GSE_LINK_IS_PAD ? GSE_BLE_NAME : GSE_BLE_TX_NAME);
    started = true;
#if GSE_LINK_IS_PAD
    startAdvertising();
#else
    restartScan();
#endif
}

#if GSE_LINK_IS_PAD

inline void GseLink::startAdvertising()
{
    server = BLEDevice::createServer();
    server->setCallbacks(&gse_ble_cb);

    BLEService *svc = server->createService(BLEUUID((uint16_t)GSE_BLE_SVC_UUID));

    /* The pad's receive characteristic: the TX node writes command frames
     * here.  WRITE_NR (write without response) as well as WRITE, so either
     * core's client can use the cheap path. */
    rxChar = svc->createCharacteristic(
        BLEUUID((uint16_t)GSE_BLE_RX_UUID),
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rxChar->setCallbacks(&gse_ble_cb);

    /* The pad's transmit characteristic: notifications carry state/events. */
    txChar = svc->createCharacteristic(
        BLEUUID((uint16_t)GSE_BLE_TX_UUID),
        BLECharacteristic::PROPERTY_NOTIFY);
    /* The 0x2902 (client characteristic configuration) descriptor is what a
     * central writes to subscribe.  Without it the TX node can connect and
     * then never receive a single byte -- which looks exactly like a dead
     * link, so it is added explicitly rather than assumed. */
    txChar->addDescriptor(new BLE2902());

    svc->start();

    BLEAdvertising *adv = server->getAdvertising();
    adv->addServiceUUID(BLEUUID((uint16_t)GSE_BLE_SVC_UUID));
    adv->setScanResponse(true);
    adv->start();
}

inline void GseLink::poll()
{
    /* Peripheral: the stack maintains the connection; report it, nothing else.
     * No blocking call here, so this can never hold up the 20 ms safe tick. */
    if (server) connected = server->getConnectedCount() > 0;
}

#else  /* ------------------------------------------------------------ central */

inline void GseLink::restartScan()
{
    if (!started) return;
    targetSeen = false;
    targetMac[0] = 0;
    lastAttemptMs = millis();
    scanStartMs = lastAttemptMs;
    armScanCallbacks();                 /* active scan: need the scan response */
    /* duration 0 = scan until stop(); the completion trampoline above is what
     * ends a fruitless scan, so this call does not block. */
    BLEDevice::getScan()->start(0, gse_ble_scan_done, false);
    scanning = true;
}

inline bool GseLink::connectAndSubscribe()
{
    if (!client) {
        client = BLEDevice::createClient();
        client->setClientCallbacks(&gse_ble_cb);
    }
    /* Address *type* is passed through from the advertisement: an ESP32 that
     * has never paired advertises a public address, but a board configured
     * otherwise must still work, and getting this wrong fails the connection
     * with no useful error. */
    if (!client->connect(BLEAddress(targetMac), (esp_ble_addr_type_t)targetType)) {
        client->disconnect();
        return false;
    }

    BLERemoteService *svc =
        client->getService(BLEUUID((uint16_t)GSE_BLE_SVC_UUID));
    if (!svc) { client->disconnect(); return false; }

    rxRemote = svc->getCharacteristic(BLEUUID((uint16_t)GSE_BLE_RX_UUID));
    BLERemoteCharacteristic *txr =
        svc->getCharacteristic(BLEUUID((uint16_t)GSE_BLE_TX_UUID));
    if (!rxRemote || !txr) { client->disconnect(); return false; }

    /* Writes the pad's CCCD: from here on its notifications land in
     * gse_ble_notify() and therefore in the ring buffer. */
    txr->registerForNotify(gse_ble_notify);
    return true;
}

inline void GseLink::poll()
{
    uint32_t now = millis();

    if (client && client->isConnected()) { connected = true; return; }
    if (connected) {                       /* a live connection went away  */
        connected = false;
        rxRemote = nullptr;
        lastAttemptMs = 0;                 /* reconnect on the next poll   */
    }

    if (scanning) {
        if (targetSeen) {                  /* found it: stop, connect      */
            BLEDevice::getScan()->stop();
            scanning = false;
            if (connectAndSubscribe()) connected = true;
            else lastAttemptMs = now;      /* back off, then scan again    */
            return;
        }
        /* Belt and braces: the completion callback usually ends the window
         * (see scanFinished()), but a stack that never fires it must not leave
         * the TX node scanning for ever while the laptop waits. */
        if (now - scanStartMs >= GSE_BLE_SCAN_MS + 1000u) {
            BLEDevice::getScan()->stop();
            scanFinished();
        }
        return;
    }

    if (lastAttemptMs == 0 || (now - lastAttemptMs) >= GSE_BLE_RETRY_MS)
        restartScan();
}

#endif /* central */

inline size_t GseLink::writeChunks(const uint8_t *p, size_t len)
{
    if (!connected) return 0;
    size_t sent = 0;
    for (size_t off = 0; off < len; off += GSE_BLE_MTU_PAYLOAD) {
        size_t n = len - off;
        if (n > GSE_BLE_MTU_PAYLOAD) n = GSE_BLE_MTU_PAYLOAD;
#if GSE_LINK_IS_PAD
        if (!txChar) return sent;
        txChar->setValue(const_cast<uint8_t *>(p + off), n);
        txChar->notify();
#else
        if (!rxRemote) return sent;
        rxRemote->writeValue(const_cast<uint8_t *>(p + off), n, false);
#endif
        sent += n;
    }
    return sent;
}

#else  /* ------------------------------------------------------- GSE_LINK_LORA
        *
        * The default transport: the DX-LR42/LR41 module on UART1.  This is the
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

#endif /* GSE_LINK_BLE */

#endif /* GSE_LINK_H */
