// vehicle_pong.ino -- ESP-NOW PING test, vehicle side, with fixed MAC addresses.
//
// Answers every PING from the ground board with a PONG (same packet, type 2,
// `sequence` and `pingSentAtMs` copied unchanged) so the ground side can print
// the round-trip time.  Pairs with ../ground_ping/ground_ping.ino.
//
// The ground board is a fixed peer: it is registered once at boot, PINGs from
// any other MAC are ignored, and every PONG goes to it.
//
// This is a stand-alone radio test, NOT the GSE firmware: it uses its own
// packet format, channel 6 and no encryption, so it cannot talk to a board
// running gse_tx_esp32s / gse_pad_esp32s3.  Flash the GSE firmware back
// before using the GUI.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace {

// The two boards, fixed (same values as GSE_NOW_TX_MAC / GSE_NOW_PAD_MAC).
constexpr uint8_t kGroundMac[ESP_NOW_ETH_ALEN] = {
    0xA4, 0xF0, 0x0F, 0x62, 0x0C, 0xB8};   // the peer (ESP32S ground)
constexpr uint8_t kVehicleMac[ESP_NOW_ETH_ALEN] = {
    0xA4, 0xF0, 0x0F, 0x61, 0x84, 0x3C};   // this board (ESP32S vehicle)

constexpr uint8_t kEspNowChannel = 6;
constexpr uint32_t kPacketMagic = 0x524F434B;  // "ROCK"
constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kTypePing = 1;
constexpr uint8_t kTypePong = 2;

struct __attribute__((packed)) Packet {
  uint32_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t reserved;
  uint32_t sequence;
  uint32_t pingSentAtMs;
};

struct ReceivedPacket {
  Packet packet;
  uint8_t sender[ESP_NOW_ETH_ALEN];
};

static_assert(sizeof(Packet) == 16, "Unexpected packet layout");

QueueHandle_t receivedPackets = nullptr;
QueueHandle_t sendResults = nullptr;
bool sendInProgress = false;
volatile uint32_t foreignPings = 0;   // valid PINGs from any other MAC

void printMac(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void onDataSent(const esp_now_send_info_t *, esp_now_send_status_t status) {
#else
void onDataSent(const uint8_t *, esp_now_send_status_t status) {
#endif
  // Unicast: "success" means the ground board's radio acknowledged the PONG.
  const bool succeeded = (status == ESP_NOW_SEND_SUCCESS);
  xQueueOverwrite(sendResults, &succeeded);
}

void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data,
                    int length) {
  if (info == nullptr || info->src_addr == nullptr || data == nullptr ||
      length != static_cast<int>(sizeof(Packet))) {
    return;
  }

  ReceivedPacket received{};
  memcpy(&received.packet, data, sizeof(received.packet));

  if (received.packet.magic != kPacketMagic ||
      received.packet.version != kProtocolVersion ||
      received.packet.type != kTypePing) {
    return;
  }
  if (memcmp(info->src_addr, kGroundMac, ESP_NOW_ETH_ALEN) != 0) {
    foreignPings = foreignPings + 1;       // not our ground board: ignore it
    return;
  }

  memcpy(received.sender, info->src_addr, ESP_NOW_ETH_ALEN);
  xQueueOverwrite(receivedPackets, &received);
}

bool addGroundPeer() {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, kGroundMac, ESP_NOW_ETH_ALEN);
  peer.channel = kEspNowChannel;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  const esp_err_t result = esp_now_add_peer(&peer);
  if (result != ESP_OK) {
    Serial.printf("Could not add ground peer: %s\n", esp_err_to_name(result));
    return false;
  }
  return true;
}

void stopWithError(const char *message) {
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    stopWithError("ERROR: could not set Wi-Fi channel");
  }

  uint8_t ownMac[ESP_NOW_ETH_ALEN] = {};
  esp_wifi_get_mac(WIFI_IF_STA, ownMac);

  Serial.println("\nESP-NOW vehicle");
  Serial.print("Vehicle MAC: ");
  printMac(ownMac);
  if (memcmp(ownMac, kVehicleMac, ESP_NOW_ETH_ALEN) != 0) {
    Serial.print("   <-- WARNING: expected ");
    printMac(kVehicleMac);
    Serial.print(" (wrong board, or kVehicleMac needs updating)");
  }
  Serial.print("\nGround MAC:  ");
  printMac(kGroundMac);
  Serial.printf("\nChannel: %u\n", kEspNowChannel);

  receivedPackets = xQueueCreate(1, sizeof(ReceivedPacket));
  sendResults = xQueueCreate(1, sizeof(bool));
  if (receivedPackets == nullptr || sendResults == nullptr) {
    stopWithError("ERROR: could not create event queues");
  }

  if (esp_now_init() != ESP_OK) {
    stopWithError("ERROR: ESP-NOW initialization failed");
  }
  if (esp_now_register_send_cb(onDataSent) != ESP_OK ||
      esp_now_register_recv_cb(onDataReceived) != ESP_OK) {
    stopWithError("ERROR: ESP-NOW callback registration failed");
  }
  if (!addGroundPeer()) {
    stopWithError("ERROR: could not add the ground board as a peer");
  }

  Serial.println("Ready; waiting for PING packets from the ground board.");
}

void loop() {
  bool succeeded = false;
  if (xQueueReceive(sendResults, &succeeded, 0) == pdTRUE) {
    sendInProgress = false;
    Serial.printf("PONG radio delivery: %s\n",
                  succeeded ? "OK (ground radio acknowledged)"
                            : "FAILED (no acknowledgement from the ground MAC)");
  }

  static uint32_t reportedForeign = 0;
  if (foreignPings != reportedForeign) {
    reportedForeign = foreignPings;
    Serial.printf("Ignored %lu PING(s) from a MAC that is not the ground board\n",
                  static_cast<unsigned long>(reportedForeign));
  }

  ReceivedPacket received{};
  if (!sendInProgress &&
      xQueueReceive(receivedPackets, &received, 0) == pdTRUE) {
    Serial.printf("PING seq=%lu from ",
                  static_cast<unsigned long>(received.packet.sequence));
    printMac(received.sender);
    Serial.println();

    received.packet.type = kTypePong;
    sendInProgress = true;
    const esp_err_t result = esp_now_send(
        kGroundMac, reinterpret_cast<const uint8_t *>(&received.packet),
        sizeof(received.packet));
    if (result == ESP_OK) {
      Serial.printf("PONG seq=%lu queued\n",
                    static_cast<unsigned long>(received.packet.sequence));
    } else {
      sendInProgress = false;
      Serial.printf("PONG queue failed: %s\n", esp_err_to_name(result));
    }
  }

  delay(1);
}
