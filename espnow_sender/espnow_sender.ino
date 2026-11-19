/*
  espnow_sender.ino - ESP-NOW bridge for the servo / ignition actuator board.

  Upload this sketch to the *controller* ESP32. The matching receiver sketch is
  ESPNOW/servo_ign/espnow_receiver/espnow_receiver.ino

  Arduino IDE:  Tools > Board > ESP32 Dev Module, Serial Monitor at 115200 baud.

  Text commands over USB serial (one per line, newline terminates):
    Test mode (timed pulses):
      1 / 2 / 3 / 4      servo0 / servo1 / both / IGN pulse
    Live mode (latched states):
      SV0_ON  SV0_OFF    FILL open / closed
      SV1_ON  SV1_OFF    VEH open / closed
      BOTH_OFF           close FILL and VEH
      IGN_ARM[:ms]       start IGN countdown (default 5000 ms)
      IGN_ON  IGN_OFF    fire / reset IGN
      ALL_OFF            force both servos closed and IGN off
      PING               replies PONG

  The desktop controller binary sends these same commands. Every ESP-NOW
  command is answered by the receiver with ACK <token> <cmd> <accepted>.

  The pulse widths and hold times below are starting values only. Tune them to
  the actual mechanism before connecting an ignition.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>     // esp_wifi_get_mac()
#include <esp_mac.h>      // esp_read_mac(), ESP_MAC_WIFI_STA
#include <esp_system.h>   // esp_random()
#include <stddef.h>
#include <string.h>

// ---------------------------------------------------------------------------
// USER CONFIGURATION - edit before flashing
// ---------------------------------------------------------------------------
// MAC address printed by the receiver sketch at boot (STA MAC).
static uint8_t RECEIVER_MAC[6] = {0xB0, 0xCB, 0xD8, 0x89, 0x4F, 0xB8}; // A4:F0:0F:61:84:3C for #1, B0:CB:D8:89:4F:B8 for #2

// PWM pulse widths sent to the receiver. DS3225 range is 500-2500 us.
static constexpr uint16_t SERVO0_TARGET_US = 500;  // FILL open position
static constexpr uint16_t SERVO1_TARGET_US = 500;  // VEH open position
static constexpr uint16_t SERVO_HOLD_MS    = 500;   // test pulse hold
static constexpr uint16_t IGN_ON_MS        = 1000;  // test IGN pulse (clamped on RX)

// Every command is sent a few times; the receiver filters duplicates by token.
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

// Wire format (must match the receiver sketch byte for byte).
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
  packet.servo0PulseUs = SERVO0_TARGET_US;
  packet.servo1PulseUs = SERVO1_TARGET_US;
  packet.crc16 = crc16Ccitt(reinterpret_cast<const uint8_t *>(&packet), CRC_OFFSET);

  for (uint8_t attempt = 0; attempt < TX_RETRIES; ++attempt)
  {
    esp_err_t err = esp_now_send(RECEIVER_MAC,
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
  if (data == nullptr || len != static_cast<int>(sizeof(AckPacket))) return;
  AckPacket ack{};
  memcpy(&ack, data, sizeof(ack));
  if (ack.magic != ACK_MAGIC) return;
  if (ack.crc16 != crc16Ccitt(data, ACK_CRC_OFFSET)) return;

  Serial.print("ACK ");
  Serial.print(ack.token);
  Serial.print(" ");
  Serial.print(ack.cmd);
  Serial.print(" ");
  Serial.println(ack.accepted ? "1" : "0");
}

static void handleLine(String line)
{
  line.trim();
  line.toUpperCase();
  if (line.length() == 0) return;

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

  Serial.println("ESP-NOW servo/IGN sender");
  if (!macConfigured(RECEIVER_MAC))
  {
    Serial.println("ERROR: set RECEIVER_MAC to the receiver's STA MAC and reflash.");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  uint8_t myMac[6] = {};
  if (readStaMac(myMac)) printMac("Sender MAC (copy into receiver sketch): ", myMac);
  else Serial.println("Could not read sender MAC");

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel = 0;      // use the ESP32's own Wi-Fi channel
  peer.encrypt = false;  // CRC + MAC filter used instead of encryption

  esp_err_t err = esp_now_add_peer(&peer);
  if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST)
  {
    Serial.print("Failed to add peer, error 0x");
    Serial.println(static_cast<int>(err), HEX);
    return;
  }

  esp_now_register_recv_cb(onAckRecv);

  Serial.println("Ready. Test: 1/2/3/4. Live: SV0_ON, SV1_ON, IGN_ARM, IGN_ON, ALL_OFF.");
  Serial.println("TEST WITH IGNITION DISCONNECTED FIRST.");
}

void loop()
{
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
