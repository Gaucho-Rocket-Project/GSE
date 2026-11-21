/*
  ground.ino - ESP-NOW bridge for the servo / ignition actuator board.

  Upload this sketch to the *controller* ESP32. The matching vehicle sketch is
  vehicle/vehicle.ino

  Arduino IDE:  Tools > Board > ESP32 Dev Module, Serial Monitor at 115200 baud.

  Text commands over USB serial (one per line, newline terminates):
    Test mode (timed pulses):
      1 / 2 / 3 / 4      servo0 / servo1 / both / IGN pulse
    Live mode (latched states):
      SV0_ON  SV0_OFF    FILL open / closed
      SV1_ON  SV1_OFF    SRV open / closed
      BOTH_OFF           close FILL and SRV
      IGN_ARM[:ms]       start IGN countdown (default 5000 ms)
      IGN_ON  IGN_OFF    fire / reset IGN
      ALL_OFF            force both servos closed and IGN off
      PING               replies PONG

  The desktop controller binary sends these same commands. Every ESP-NOW
  command is answered by the vehicle with ACK <token> <cmd> <accepted>.

  The pulse widths and hold times below are starting values only. Tune them to
  the actual mechanism before connecting an ignition.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>     // esp_wifi_get_mac()
#include <esp_mac.h>      // esp_read_mac(), ESP_MAC_WIFI_STA
#include <esp_system.h>   // esp_random()
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stddef.h>
#include <string.h>

// ---------------------------------------------------------------------------
// USER CONFIGURATION - edit before flashing
// ---------------------------------------------------------------------------
// MAC address printed by the vehicle sketch at boot (STA MAC).
static uint8_t VEHICLE_MAC[6] = {0xB0, 0xCB, 0xD8, 0x89, 0x4F, 0xB8}; // A4:F0:0F:61:84:3C for #1, B0:CB:D8:89:4F:B8 for #2

// PWM pulse widths sent to the vehicle. DS3225 range is 500-2500 us.
static uint16_t servo0TargetUs = 1800;  // FILL open position
static uint16_t servo1TargetUs = 1800;  // SRV open position
static constexpr uint16_t SERVO_HOLD_MS    = 500;   // test pulse hold
static constexpr uint16_t IGN_ON_MS        = 1000;  // test IGN pulse (clamped on RX)

// Every command is sent a few times; the vehicle filters duplicates by token.
static constexpr uint8_t  TX_RETRIES       = 3;
static constexpr uint16_t TX_RETRY_GAP_MS  = 120;
// ---------------------------------------------------------------------------

enum : uint8_t {
  CMD_SERVO0 = 1, CMD_SERVO1 = 2, CMD_BOTH = 3, CMD_IGN = 4,
  CMD_SV0_OFF = 10, CMD_SV0_ON = 11,
  CMD_SV1_OFF = 12, CMD_SV1_ON = 13,
  CMD_BOTH_OFF = 14, CMD_BOTH_ON = 15,
  CMD_IGN_OFF = 16, CMD_IGN_ARM = 17, CMD_IGN_ON = 18,
  CMD_ALL_OFF = 19
};

// Wire format (must match the vehicle sketch byte for byte).
static constexpr uint32_t PACKET_MAGIC = 0x314B4352U;  // "RCK1" little endian
static constexpr uint32_t ACK_MAGIC    = 0x314B4341U;  // "ACK1" little endian

struct __attribute__((packed)) CommandPacket {
  uint32_t magic;
  uint32_t token;
  uint8_t  cmd;
  uint8_t  reserved;        // 0 = test pulse, 1 = latched live state
  uint16_t holdMs;
  uint16_t servo0PulseUs;
  uint16_t servo1PulseUs;
  uint16_t crc16;
};

struct __attribute__((packed)) AckPacket {
  uint32_t magic;
  uint32_t token;
  uint8_t  cmd;
  uint8_t  accepted;
  uint16_t crc16;
};

static constexpr size_t TELEMETRY_MAX = 200;
static constexpr uint32_t TELEMETRY_MAGIC = 0x314D4C54U;
struct __attribute__((packed)) TelemetryPacket {
  uint32_t magic;
  uint32_t sequence;
  uint32_t uptimeMs;
  uint8_t length;
  char data[TELEMETRY_MAX];
  uint16_t crc16;
};
static_assert(sizeof(TelemetryPacket) <= 250, "ESP-NOW payload too large");
// Receive callbacks run in the Wi-Fi task. Print complete records in loop().
struct ReceivedMessage { bool telemetry; AckPacket ack; TelemetryPacket sample; };
static QueueHandle_t receivedQueue = nullptr;
static constexpr size_t CRC_OFFSET = offsetof(CommandPacket, crc16);
static constexpr size_t ACK_CRC_OFFSET = offsetof(AckPacket, crc16);

static String rxLine;   // USB serial line buffer

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection/XOROUT.
static uint16_t crc16Ccitt(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i)
  {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit)
    {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                          : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

static bool macConfigured(const uint8_t *mac)
{
  bool anyNonZero = false;
  bool allOnes = true;
  for (uint8_t i = 0; i < 6; ++i)
  {
    if (mac[i] != 0) anyNonZero = true;
    if (mac[i] != 0xFF) allOnes = false;
  }
  return anyNonZero && !allOnes;
}

// WiFi.macAddress() can print 00:00:00:00:00:00 on Arduino-ESP32 3.x when the
// STA interface has not started yet. Read the driver directly instead, with
// the eFuse-derived station MAC as a fallback.
static bool allZeroMac(const uint8_t *mac)
{
  for (uint8_t i = 0; i < 6; ++i)
  {
    if (mac[i] != 0) return false;
  }
  return true;
}

static bool readStaMac(uint8_t mac[6])
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  unsigned long startWait = millis();
  while (!WiFi.STA.started() && (millis() - startWait) < 2000) delay(20);
#endif
  if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK && !allZeroMac(mac)) return true;
  return esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK && !allZeroMac(mac);
}

static void printMac(const char *label, const uint8_t mac[6])
{
  Serial.print(label);
  for (uint8_t i = 0; i < 6; ++i)
  {
    if (i > 0) Serial.print(':');
    if (mac[i] < 0x10) Serial.print('0');
    Serial.print(mac[i], HEX);
  }
  Serial.println();
}

static void sendCommand(uint8_t cmd, uint16_t holdMs = 0, uint8_t mode = 0)
{
  uint16_t hold = (holdMs != 0) ? holdMs
                : ((cmd == CMD_IGN) ? IGN_ON_MS : SERVO_HOLD_MS);

  CommandPacket packet{};
  packet.magic = PACKET_MAGIC;
  packet.token = esp_random();
  if (packet.token == 0) packet.token = 1;   // 0 reserved as "no token"
  packet.cmd = cmd;
  packet.reserved = mode;
  packet.holdMs = hold;
  packet.servo0PulseUs = servo0TargetUs;
  packet.servo1PulseUs = servo1TargetUs;
  packet.crc16 = crc16Ccitt(reinterpret_cast<const uint8_t *>(&packet), CRC_OFFSET);

  for (uint8_t attempt = 0; attempt < TX_RETRIES; ++attempt)
  {
    esp_err_t err = esp_now_send(VEHICLE_MAC,
                                 reinterpret_cast<const uint8_t *>(&packet),
                                 sizeof(packet));
    if (err != ESP_OK)
    {
      Serial.print("ERR send failed 0x");
      Serial.println(static_cast<int>(err), HEX);
    }
    if (attempt + 1 < TX_RETRIES) delay(TX_RETRY_GAP_MS);
  }
  Serial.print("SENT ");
  Serial.print(cmd);
  Serial.print(" ");
  Serial.println(packet.token);
}

// ESP-NOW receive callback signature changed between Arduino-ESP32 2.x and 3.x.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onAckRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  (void)info;
#else
void onAckRecv(const uint8_t *mac, const uint8_t *data, int len)
{
  (void)mac;
#endif
  const uint8_t *src =
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      info ? info->src_addr : nullptr;
#else
      mac;
#endif
  if (!src || memcmp(src, VEHICLE_MAC, 6) != 0 || !data || !receivedQueue) return;
  ReceivedMessage msg{};
  if (len == static_cast<int>(sizeof(AckPacket))) {
    memcpy(&msg.ack, data, sizeof(msg.ack));
    if (msg.ack.magic != ACK_MAGIC || msg.ack.crc16 != crc16Ccitt(data, ACK_CRC_OFFSET)) return;
  } else if (len == static_cast<int>(sizeof(TelemetryPacket))) {
    memcpy(&msg.sample, data, sizeof(msg.sample));
    if (msg.sample.magic != TELEMETRY_MAGIC || msg.sample.length == 0 || msg.sample.length > TELEMETRY_MAX ||
        msg.sample.crc16 != crc16Ccitt(data, offsetof(TelemetryPacket, crc16))) return;
    for (size_t i = 0; i < msg.sample.length; ++i)
      if (msg.sample.data[i] < 32 || msg.sample.data[i] > 126) return;
    msg.telemetry = true;
  } else return;
  xQueueSend(receivedQueue, &msg, 0);

}

static void handleLine(String line)
{
  line.trim();
  line.toUpperCase();
  if (line.length() == 0) return;

  // Optional per-command pulse targets: COMMAND:<fill>,<srv>. Validated as
  // a pair before sending; legacy commands retain the most recent targets.
  if (!line.startsWith("IGN_ARM") && line.indexOf(':') >= 0) {
    int colon = line.indexOf(':');
    int comma = line.indexOf(',', colon + 1);
    if (comma < 0) { Serial.println("ERR invalid targets"); return; }
    String a = line.substring(colon + 1, comma), b = line.substring(comma + 1);
    if (!a.length() || !b.length()) { Serial.println("ERR invalid targets"); return; }
    for (size_t i = 0; i < a.length(); ++i) if (!isDigit(a[i])) { Serial.println("ERR invalid targets"); return; }
    for (size_t i = 0; i < b.length(); ++i) if (!isDigit(b[i])) { Serial.println("ERR invalid targets"); return; }
    long fill = a.toInt(), srv = b.toInt();
    if (fill < 500 || fill > 2500 || srv < 500 || srv > 2500) { Serial.println("ERR targets out of range"); return; }
    servo0TargetUs = fill; servo1TargetUs = srv;
    line = line.substring(0, colon);
  }

  // Legacy single-key test commands.
  if (line.length() == 1)
  {
    switch (line.charAt(0))
    {
      case '1': sendCommand(CMD_SERVO0); return;
      case '2': sendCommand(CMD_SERVO1); return;
      case '3': sendCommand(CMD_BOTH);   return;
      case '4': sendCommand(CMD_IGN);    return;
      default: break;
    }
  }

  if      (line == "SV0_OFF")  sendCommand(CMD_SV0_OFF, 0, 1);
  else if (line == "SV0_ON")   sendCommand(CMD_SV0_ON, 0, 1);
  else if (line == "SV1_OFF")  sendCommand(CMD_SV1_OFF, 0, 1);
  else if (line == "SV1_ON")   sendCommand(CMD_SV1_ON, 0, 1);
  else if (line == "BOTH_OFF") sendCommand(CMD_BOTH_OFF, 0, 1);
  else if (line == "BOTH_ON")  sendCommand(CMD_BOTH_ON, 0, 1);
  else if (line == "IGN_OFF")  sendCommand(CMD_IGN_OFF, 0, 1);
  else if (line == "IGN_ON")   sendCommand(CMD_IGN_ON, 0, 1);
  else if (line == "ALL_OFF")  sendCommand(CMD_ALL_OFF, 0, 1);
  else if (line == "PING")     Serial.println("PONG");
  else if (line.startsWith("IGN_ARM"))
  {
    uint16_t ms = 5000;
    int colon = line.indexOf(':');
    if (colon >= 0 && colon + 1 < line.length())
    {
      long requested = line.substring(colon + 1).toInt();
      if (requested > 0 && requested <= 60000) ms = static_cast<uint16_t>(requested);
    }
    sendCommand(CMD_IGN_ARM, ms, 1);
  }
  else Serial.println("ERR unknown command");
}

void setup()
{
  Serial.begin(115200);
  delay(50);

  Serial.println("ESP-NOW servo/IGN ground");
  if (!macConfigured(VEHICLE_MAC))
  {
    Serial.println("ERROR: set VEHICLE_MAC to the vehicle's STA MAC and reflash.");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  uint8_t myMac[6] = {};
  if (readStaMac(myMac)) printMac("Ground MAC (copy into vehicle sketch): ", myMac);
  else Serial.println("Could not read ground MAC");

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, VEHICLE_MAC, 6);
  peer.channel = 0;      // use the ESP32's own Wi-Fi channel
  peer.encrypt = false;  // CRC + MAC filter used instead of encryption

  esp_err_t err = esp_now_add_peer(&peer);
  if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST)
  {
    Serial.print("Failed to add peer, error 0x");
    Serial.println(static_cast<int>(err), HEX);
    return;
  }

  receivedQueue = xQueueCreate(16, sizeof(ReceivedMessage));
  if (!receivedQueue) { Serial.println("ERROR: receive queue allocation failed"); return; }
  esp_now_register_recv_cb(onAckRecv);

  Serial.println("Ready. Test: 1/2/3/4. Live: SV0_ON, SV1_ON, IGN_ARM, IGN_ON, ALL_OFF.");
  Serial.println("TEST WITH IGNITION DISCONNECTED FIRST.");
}

void loop()
{
  ReceivedMessage msg{};
  while (receivedQueue && xQueueReceive(receivedQueue, &msg, 0) == pdTRUE) {
    if (msg.telemetry) {
      Serial.print("TLM "); Serial.print(msg.sample.sequence); Serial.print(" ");
      Serial.print(msg.sample.uptimeMs); Serial.print(" ");
      Serial.write(reinterpret_cast<const uint8_t *>(msg.sample.data), msg.sample.length);
      Serial.println();
    } else {
      Serial.print("ACK "); Serial.print(msg.ack.token); Serial.print(" ");
      Serial.print(msg.ack.cmd); Serial.print(" "); Serial.println(msg.ack.accepted ? "1" : "0");
    }
  }

  while (Serial.available() > 0)
  {
    char incoming = static_cast<char>(Serial.read());
    if (incoming == '\n' || incoming == '\r')
    {
      handleLine(rxLine);
      rxLine = "";
    }
    else
    {
      rxLine += incoming;
      if (rxLine.length() > 63)
      {
        Serial.println("ERR command too long");
        rxLine = "";
      }
    }
  }
}
