// -*- mode: c++ -*-
/*
 * BLEDevice.h / BLE2902.h / BLEUtils.h / BLEAddress.h  --  stand-ins for the
 * arduino-esp32 "BLE" library, used only by tests/esp32_syntax_check.sh
 * ---------------------------------------------------------------------------
 * These declarations mirror the *union* of the arduino-esp32 2.0.x and 3.x BLE
 * APIs -- the two cores this repository supports -- restricted to what
 * firmware/src/link.h actually calls.  Where the two cores genuinely differ,
 * the difference is called out below, because the firmware has to compile
 * against both and the point of this stub is to catch it when it does not:
 *
 *   BLEDevice::init()          void   in 2.x  | bool  in 3.x   -> call as a
 *                              statement; the stub returns bool and the result
 *                              may be discarded, which is legal either way.
 *   BLEScan::start()           returns BLEScanResults by value in 2.x, by
 *                              pointer in 3.x.  The firmware calls it as a
 *                              statement and never reads the result (the
 *                              device arrives through onResult()), so the stub
 *                              declares both forms returning void -- which is
 *                              exactly the discipline the firmware must keep.
 *   BLEScanResults             complete, because the scan-completion callback
 *                              takes one by value; nothing else about it is
 *                              modelled, on purpose.
 *   BLEAdvertisedDevice::getName()  std::string in 2.x | String in 3.x.  The
 *                              stub returns a String; the firmware only uses
 *                              .c_str(), which both provide.
 *
 * It is NOT a simulator: every body is empty or returns a default.  Its job is
 * to make a typo, a wrong argument count or an API that only exists in one of
 * the two cores fail in a second on a workstation.
 */
#ifndef BLE_DEVICE_H_SYNTAX_STUB
#define BLE_DEVICE_H_SYNTAX_STUB

#include <stdint.h>
#include <stddef.h>
#include "Arduino.h"

/* 2.x passes this as the address-type argument to BLEClient::connect() and
 * gets it from BLEAdvertisedDevice::getAddressType(); 3.x uses the same enum
 * name with the same members. */
typedef enum {
    BLE_ADDR_TYPE_PUBLIC = 0x00,
    BLE_ADDR_TYPE_RANDOM = 0x01,
    BLE_ADDR_TYPE_PUBLIC_RPA = 0x02,
    BLE_ADDR_TYPE_RANDOM_RPA = 0x03
} esp_ble_addr_type_t;

typedef uint8_t esp_bd_addr_t[6];

class BLEUUID {
public:
    BLEUUID() {}
    BLEUUID(uint16_t) {}
    BLEUUID(uint32_t) {}
    BLEUUID(const char *) {}
    BLEUUID(const String &) {}
    bool equals(BLEUUID) { return false; }
    String toString() { return String(); }
};

class BLEAddress {
public:
    BLEAddress() {}
    BLEAddress(const char *) {}
    BLEAddress(const String &, uint8_t = 0) {}
    BLEAddress(esp_bd_addr_t, uint8_t = 0) {}
    String toString() { return String(); }
    esp_bd_addr_t *getNative() { return nullptr; }
};

class BLECharacteristic;

class BLEDescriptor {
public:
    BLEDescriptor(const char *) {}
    BLEDescriptor(BLEUUID) {}
    void setValue(const uint8_t *, size_t) {}
    BLEUUID getUUID() { return BLEUUID(); }
};

/* 0x2902 client characteristic configuration: what a central writes to
 * subscribe to notifications.  Present in both cores. */
class BLE2902 : public BLEDescriptor {
public:
    BLE2902() : BLEDescriptor(BLEUUID((uint16_t)0x2902)) {}
    bool getNotifications() { return true; }
    void setNotifications(bool) {}
};

class BLECharacteristicCallbacks {
public:
    virtual ~BLECharacteristicCallbacks() {}
    virtual void onWrite(BLECharacteristic *) {}
    virtual void onNotify(BLECharacteristic *) {}
};

class BLECharacteristic {
public:
    static const uint32_t PROPERTY_READ      = 1 << 0;
    static const uint32_t PROPERTY_WRITE     = 1 << 1;
    static const uint32_t PROPERTY_NOTIFY    = 1 << 2;
    static const uint32_t PROPERTY_INDICATE  = 1 << 4;
    static const uint32_t PROPERTY_WRITE_NR  = 1 << 5;

    BLECharacteristic(const char *, uint32_t) {}
    BLECharacteristic(BLEUUID, uint32_t) {}

    void setCallbacks(BLECharacteristicCallbacks *) {}
    void addDescriptor(BLEDescriptor *) {}
    void setValue(const uint8_t *, size_t) {}
    void setValue(const String &) {}
    void notify(bool = true) {}
    uint8_t *getData() { return nullptr; }
    size_t getLength() { return 0; }
    String getValue() { return String(); }
    uint16_t getHandle() { return 0; }
};

class BLEService {
public:
    BLECharacteristic *createCharacteristic(const char *, uint32_t) { return nullptr; }
    BLECharacteristic *createCharacteristic(BLEUUID, uint32_t) { return nullptr; }
    BLECharacteristic *getCharacteristic(const char *) { return nullptr; }
    bool start() { return true; }
    void stop() {}
};

class BLEAdvertising {
public:
    void setName(String) {}
    void addServiceUUID(BLEUUID) {}
    void addServiceUUID(const char *) {}
    void setScanResponse(bool) {}
    void start() {}
    void stop() {}
};

class BLEScanResults;

class BLEScan {
public:
    void setActiveScan(bool) {}
    void setInterval(uint16_t) {}
    void setWindow(uint16_t) {}
    void setAdvertisedDeviceCallbacks(class BLEAdvertisedDeviceCallbacks *,
                                      bool = false, bool = true) {}
    /* The form the firmware uses: duration 0 = until stop(), completion
     * callback in the 2.0.x/3.0.x shape, `is_continue` false.  Written as a
     * statement, so the (core-dependent) return value never matters. */
    void start(uint32_t, void (*)(BLEScanResults), bool = false) {}
    void stop() {}
    bool isScanning() { return false; }
};

class BLEServerCallbacks {
public:
    virtual ~BLEServerCallbacks() {}
    virtual void onConnect(class BLEServer *) {}
    virtual void onDisconnect(class BLEServer *) {}
};

class BLEServer {
public:
    void setCallbacks(BLEServerCallbacks *) {}
    BLEService *createService(const char *) { return nullptr; }
    BLEService *createService(BLEUUID, uint32_t = 15, uint8_t = 0) { return nullptr; }
    BLEAdvertising *getAdvertising() { return nullptr; }
    void startAdvertising() {}
    uint32_t getConnectedCount() { return 0; }
};

class BLERemoteCharacteristic;

class BLERemoteService {
public:
    BLERemoteCharacteristic *getCharacteristic(const char *) { return nullptr; }
    BLERemoteCharacteristic *getCharacteristic(uint16_t) { return nullptr; }
    BLERemoteCharacteristic *getCharacteristic(BLEUUID) { return nullptr; }
};

/* One signature for the notification handler in both cores: 2.x spells it
 * std::function<void(BLERemoteCharacteristic*, uint8_t*, size_t, bool)>, 3.x
 * the same, so a plain function pointer converts in both. */
typedef void (*notify_callback)(BLERemoteCharacteristic *, uint8_t *, size_t, bool);

class BLERemoteCharacteristic {
public:
    void registerForNotify(notify_callback, bool = true, bool = true) {}
    bool writeValue(uint8_t *, size_t, bool = false) { return true; }
    bool writeValue(uint8_t, bool = false) { return true; }
};

class BLEClientCallbacks {
public:
    virtual ~BLEClientCallbacks() {}
    virtual void onConnect(class BLEClient *) {}
    virtual void onDisconnect(class BLEClient *) {}
};

class BLEClient {
public:
    void setClientCallbacks(BLEClientCallbacks *) {}
    bool connect(BLEAddress, esp_ble_addr_type_t = BLE_ADDR_TYPE_PUBLIC) { return false; }
    void disconnect() {}
    bool isConnected() { return false; }
    BLERemoteService *getService(const char *) { return nullptr; }
    BLERemoteService *getService(BLEUUID) { return nullptr; }
    uint16_t getMTU() { return 23; }
    bool setMTU(uint16_t) { return true; }
};

class BLEAdvertisedDeviceCallbacks {
public:
    virtual ~BLEAdvertisedDeviceCallbacks() {}
    virtual void onResult(class BLEAdvertisedDevice) = 0;
};

class BLEAdvertisedDevice {
public:
    BLEAddress getAddress() { return BLEAddress(); }
    uint8_t getAddressType() { return 0; }
    String getName() { return String(); }
    bool haveName() { return false; }
    bool isAdvertisingService(BLEUUID) { return false; }
    uint8_t *getData() { return nullptr; }
    size_t getLength() { return 0; }
};

/* Complete (not just forward-declared): BLEScan::start()'s completion callback
 * takes one by value in both cores, so an incomplete type will not compile.  It
 * deliberately has no accessors the firmware might be tempted to use -- the
 * whole point is that the results are never inspected (see link.h). */
class BLEScanResults {
public:
    int getCount() { return 0; }
};

class BLEDevice {
public:
    static bool init(String = String()) { return true; }
    static void deinit() {}
    static BLEScan *getScan() { return nullptr; }
    static BLEServer *createServer() { return nullptr; }
    static BLEClient *createClient() { return nullptr; }
    static BLEAdvertising *getAdvertising() { return nullptr; }
    static void startAdvertising() {}
    static uint16_t getMTU() { return 23; }
    static bool setMTU(uint16_t) { return true; }
};

#endif /* BLE_DEVICE_H_SYNTAX_STUB */
