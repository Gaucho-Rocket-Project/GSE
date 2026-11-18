/*
  espnow_sender.ino - ESP-NOW controller for the servo / ignition actuator board.

  Upload this sketch to the *controller* ESP32. The matching receiver sketch is
  ESPNOW/servo_ign/espnow_receiver/espnow_receiver.ino

  Arduino IDE:  Tools > Board > ESP32 Dev Module, Serial Monitor at 115200 baud.

  Commands (type in Serial Monitor and press Enter):
    1 = actuate servo0 (GPIO 18 on receiver)
    2 = actuate servo1 (GPIO 5 on receiver)
    3 = actuate servo0 and servo1 together
    4 = IGN (GPIO 32 on receiver, active HIGH)

  The pulse widths and hold times below are starting values only. Tune them to
  the actual mechanism before connecting an ignition.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_system.h>   // esp_random()
#include <stddef.h>
#include <string.h>

// ---------------------------------------------------------------------------
// USER CONFIGURATION - edit before flashing
// ---------------------------------------------------------------------------
// MAC address printed by the receiver sketch at boot (STA MAC).
static uint8_t RECEIVER_MAC[6] = {0x7C, 0x9E, 0xBD, 0x12, 0x34, 0x56};

// PWM pulse widths sent to the receiver. DS3225 range is 500-2500 us.
static constexpr uint16_t SERVO0_TARGET_US = 2000;  // fired/target position
static constexpr uint16_t SERVO1_TARGET_US = 2000;
static constexpr uint16_t SERVO_HOLD_MS    = 500;   // time at target before return

static constexpr uint16_t IGN_ON_MS        = 1000;  // receiver clamps to 2000 ms max

// Every command is sent a few times; the receiver filters duplicates by token.
static constexpr uint8_t  TX_RETRIES       = 3;
static constexpr uint16_t TX_RETRY_GAP_MS  = 120;
// ---------------------------------------------------------------------------

enum : uint8_t {
  CMD_SERVO0 = 1,
  CMD_SERVO1 = 2,
  CMD_BOTH   = 3,
  CMD_IGN    = 4
};

// Wire format (must match the receiver sketch byte for byte).
static constexpr uint32_t PACKET_MAGIC = 0x314B4352U;  // "RCK1" little endian

struct __attribute__((packed)) CommandPacket {
  uint32_t magic;           // PACKET_MAGIC
  uint32_t token;           // random, used by receiver to drop retransmissions
  uint8_t  cmd;             // CMD_*
  uint8_t  reserved;        // 0
  uint16_t holdMs;          // ms at target / IGN on-time
  uint16_t servo0PulseUs;   // 500-2500
  uint16_t servo1PulseUs;   // 500-2500
  uint16_t crc16;           // CRC-16/CCITT over all preceding bytes
};

static constexpr size_t CRC_OFFSET = offsetof(CommandPacket, crc16);

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

static void sendCommand(uint8_t cmd)
{
  CommandPacket packet{};
  packet.magic = PACKET_MAGIC;
  packet.token = esp_random();
  if (packet.token == 0) packet.token = 1;   // 0 reserved as "no token"
  packet.cmd = cmd;
  packet.reserved = 0;
  packet.holdMs = (cmd == CMD_IGN) ? IGN_ON_MS : SERVO_HOLD_MS;
  packet.servo0PulseUs = SERVO0_TARGET_US;
  packet.servo1PulseUs = SERVO1_TARGET_US;
  packet.crc16 = crc16Ccitt(reinterpret_cast<const uint8_t *>(&packet), CRC_OFFSET);

  const char *name = "?";
  if (cmd == CMD_SERVO0) name = "servo0";
  else if (cmd == CMD_SERVO1) name = "servo1";
  else if (cmd == CMD_BOTH) name = "both servos";
  else if (cmd == CMD_IGN) name = "IGN";

  for (uint8_t attempt = 0; attempt < TX_RETRIES; ++attempt)
  {
    esp_err_t err = esp_now_send(RECEIVER_MAC,
                                 reinterpret_cast<const uint8_t *>(&packet),
                                 sizeof(packet));
    if (err != ESP_OK)
    {
      Serial.print("esp_now_send failed, error 0x");
      Serial.println(static_cast<int>(err), HEX);
    }
    if (attempt + 1 < TX_RETRIES) delay(TX_RETRY_GAP_MS);
  }
  Serial.print("Sent ");
  Serial.print(name);
  Serial.print(" (token ");
  Serial.print(packet.token);
  Serial.println(")");
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

  Serial.print("Sender MAC (copy into receiver sketch): ");
  Serial.println(WiFi.macAddress());

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

  Serial.println("Ready. Commands: 1=servo0  2=servo1  3=both  4=IGN");
  Serial.println("TEST WITH IGNITION DISCONNECTED FIRST.");
}

void loop()
{
  if (Serial.available() > 0)
  {
    int incoming = Serial.read();
    while (Serial.available() > 0) Serial.read();   // flush line ending/extra keys

    switch (incoming)
    {
      case '1': sendCommand(CMD_SERVO0); break;
      case '2': sendCommand(CMD_SERVO1); break;
      case '3': sendCommand(CMD_BOTH);   break;
      case '4': sendCommand(CMD_IGN);    break;
      default:
        Serial.println("Use 1=servo0, 2=servo1, 3=both, 4=IGN");
        break;
    }
  }
}
