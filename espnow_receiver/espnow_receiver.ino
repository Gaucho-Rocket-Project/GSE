/*
  espnow_receiver.ino - ESP-NOW servo / ignition actuator board.

  Upload this sketch to the *actuator* ESP32:
    servo0 signal : GPIO 18
    servo1 signal : GPIO 5
    IGN           : GPIO 32, active HIGH

  Servos are the DS3225 digital servos from the 52Pi D-0001 wiki:
    4.8-6.8 V supply, 500-2500 us pulse, 50-330 Hz, up to 2.9 A stall each.
    Power them from a separate BEC (>= 6 A peak for two servos) and connect
    the BEC ground to the ESP32 ground. Never power them from the ESP32 5 V pin.

  Arduino IDE:  Tools > Board > ESP32 Dev Module, Serial Monitor at 115200 baud.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>     // esp_wifi_set_ps()
#include <stddef.h>
#include <string.h>

// ---------------------------------------------------------------------------
// USER CONFIGURATION - edit before flashing
// ---------------------------------------------------------------------------
// MAC address printed by the sender sketch at boot (STA MAC).
// With ENFORCE_SENDER_MAC = true, packets from any other board are ignored.
static uint8_t SENDER_MAC[6] = {0x7C, 0x9E, 0xBD, 0x12, 0x34, 0x56};
static constexpr bool ENFORCE_SENDER_MAC = true;  // false only for bench testing

static constexpr uint8_t  SERVO0_PIN = 18;
static constexpr uint8_t  SERVO1_PIN = 5;
static constexpr uint8_t  IGN_PIN    = 32;        // active HIGH

// LEDC channels (only used by Arduino-ESP32 2.x; 3.x assigns channels itself).
static constexpr uint8_t  SERVO0_CHANNEL = 0;
static constexpr uint8_t  SERVO1_CHANNEL = 1;

static constexpr uint32_t PWM_FREQ_HZ       = 50;   // standard RC servo frame
static constexpr uint8_t  PWM_RESOLUTION_BITS = 16;
static constexpr uint16_t PWM_PERIOD_US     = 1000000UL / PWM_FREQ_HZ;  // 20000 us

static constexpr uint16_t SERVO_MIN_US      = 500;
static constexpr uint16_t SERVO_MAX_US      = 2500;
static constexpr uint16_t SERVO_HOME_US     = 1000; // rest/retracted position
static constexpr uint16_t MAX_SERVO_HOLD_MS = 5000; // hard cap on servo target time
static constexpr uint16_t MAX_IGN_ON_MS     = 2000; // hard cap on IGN on-time
// ---------------------------------------------------------------------------

enum : uint8_t {
  CMD_SERVO0 = 1,
  CMD_SERVO1 = 2,
  CMD_BOTH   = 3,
  CMD_IGN    = 4
};

// Wire format (must match the sender sketch byte for byte).
static constexpr uint32_t PACKET_MAGIC = 0x314B4352U;  // "RCK1" little endian

struct __attribute__((packed)) CommandPacket {
  uint32_t magic;
  uint32_t token;
  uint8_t  cmd;
  uint8_t  reserved;
  uint16_t holdMs;
  uint16_t servo0PulseUs;
  uint16_t servo1PulseUs;
  uint16_t crc16;
};

static constexpr size_t CRC_OFFSET = offsetof(CommandPacket, crc16);
static constexpr uint8_t SEEN_TOKEN_COUNT = 8;

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

// LEDC wrapper: Arduino-ESP32 3.x addresses a channel by pin, 2.x by channel.
static bool pwmAttach(uint8_t pin, uint8_t channel, uint32_t freq, uint8_t resolution)
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  return ledcAttach(pin, freq, resolution);
#else
  double actual = ledcSetup(channel, freq, resolution);
  ledcAttachPin(pin, channel);
  return actual > 0.0;
#endif
}

static void pwmWrite(uint8_t pin, uint8_t channel, uint32_t duty)
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

static uint32_t pulseToDuty(uint16_t pulseUs)
{
  return (static_cast<uint32_t>(pulseUs) << PWM_RESOLUTION_BITS) / PWM_PERIOD_US;
}

static bool validPulse(uint16_t pulseUs)
{
  return pulseUs >= SERVO_MIN_US && pulseUs <= SERVO_MAX_US;
}

static void setServoHome()
{
  pwmWrite(SERVO0_PIN, SERVO0_CHANNEL, pulseToDuty(SERVO_HOME_US));
  pwmWrite(SERVO1_PIN, SERVO1_CHANNEL, pulseToDuty(SERVO_HOME_US));
}

// ---- Action state machine (non-blocking) ----------------------------------
enum class Phase : uint8_t { IDLE, SERVO_HOLD, IGN_HOLD };

static Phase    phase          = Phase::IDLE;
static uint32_t phaseStartMs   = 0;
static uint32_t phaseDuration  = 0;
static bool     servo0Active   = false;
static bool     servo1Active   = false;

static void startAction(const CommandPacket &packet)
{
  if (phase != Phase::IDLE)
  {
    Serial.println("BUSY: command ignored");
    return;
  }

  if (packet.cmd == CMD_SERVO0 || packet.cmd == CMD_SERVO1 || packet.cmd == CMD_BOTH)
  {
    if (!validPulse(packet.servo0PulseUs) || !validPulse(packet.servo1PulseUs)) return;

    servo0Active = (packet.cmd == CMD_SERVO0 || packet.cmd == CMD_BOTH);
    servo1Active = (packet.cmd == CMD_SERVO1 || packet.cmd == CMD_BOTH);

    pwmWrite(SERVO0_PIN, SERVO0_CHANNEL,
             pulseToDuty(servo0Active ? packet.servo0PulseUs : SERVO_HOME_US));
    pwmWrite(SERVO1_PIN, SERVO1_CHANNEL,
             pulseToDuty(servo1Active ? packet.servo1PulseUs : SERVO_HOME_US));

    phase = Phase::SERVO_HOLD;
    phaseStartMs = millis();
    phaseDuration = min(packet.holdMs, MAX_SERVO_HOLD_MS);

    Serial.print("Actuating ");
    Serial.println(servo0Active ? (servo1Active ? "servo0 + servo1" : "servo0") : "servo1");
  }
  else if (packet.cmd == CMD_IGN)
  {
    // Servos stay at home while the igniter is fired.
    setServoHome();
    servo0Active = false;
    servo1Active = false;

    digitalWrite(IGN_PIN, HIGH);
    phase = Phase::IGN_HOLD;
    phaseStartMs = millis();
    phaseDuration = (packet.holdMs == 0) ? 1 : min(packet.holdMs, MAX_IGN_ON_MS);

    Serial.print("IGN HIGH for ");
    Serial.print(phaseDuration);
    Serial.println(" ms");
  }
}

// ---- Duplicate rejection ---------------------------------------------------
static uint32_t seenTokens[SEEN_TOKEN_COUNT] = {};
static uint8_t  seenIndex = 0;

static bool tokenAlreadySeen(uint32_t token)
{
  for (uint8_t i = 0; i < SEEN_TOKEN_COUNT; ++i)
  {
    if (seenTokens[i] == token) return true;
  }
  return false;
}

static void rememberToken(uint32_t token)
{
  seenTokens[seenIndex] = token;
  seenIndex = (seenIndex + 1) % SEEN_TOKEN_COUNT;
}

// ESP-NOW receive callback signature changed between Arduino-ESP32 2.x and 3.x.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  const uint8_t *src = (info != nullptr) ? info->src_addr : nullptr;
#else
void onDataRecv(const uint8_t *src, const uint8_t *data, int len)
{
#endif
  if (src == nullptr || data == nullptr || len != static_cast<int>(sizeof(CommandPacket)))
  {
    return;
  }

  if (ENFORCE_SENDER_MAC && memcmp(src, SENDER_MAC, 6) != 0) return;

  CommandPacket packet{};
  memcpy(&packet, data, sizeof(packet));

  if (packet.magic != PACKET_MAGIC) return;
  if (packet.crc16 != crc16Ccitt(data, CRC_OFFSET)) return;
  if (packet.token == 0 || tokenAlreadySeen(packet.token)) return;
  if (packet.reserved != 0) return;

  rememberToken(packet.token);
  startAction(packet);
}

void setup()
{
  Serial.begin(115200);
  delay(50);

  // Ignition output is forced LOW before anything else and stays LOW except
  // during a validated, time-limited CMD_IGN action.
  pinMode(IGN_PIN, OUTPUT);
  digitalWrite(IGN_PIN, LOW);

  if (!pwmAttach(SERVO0_PIN, SERVO0_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS) ||
      !pwmAttach(SERVO1_PIN, SERVO1_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS))
  {
    Serial.println("LEDC setup failed");
  }
  setServoHome();

  if (ENFORCE_SENDER_MAC && !macConfigured(SENDER_MAC))
  {
    Serial.println("WARNING: SENDER_MAC not set - all commands will be rejected.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);   // ESP-NOW reception is more reliable awake

  Serial.print("Receiver MAC (copy into sender sketch): ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
    return;
  }

  if (macConfigured(SENDER_MAC))
  {
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, SENDER_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST)
    {
      Serial.print("Failed to add peer, error 0x");
      Serial.println(static_cast<int>(err), HEX);
    }
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Ready. TEST WITH IGNITION DISCONNECTED FIRST.");
}

void loop()
{
  // Return servos to their home pulse once the hold time has elapsed.
  if (phase == Phase::SERVO_HOLD && (millis() - phaseStartMs) >= phaseDuration)
  {
    setServoHome();
    servo0Active = false;
    servo1Active = false;
    phase = Phase::IDLE;
    Serial.println("Servos back at home");
  }

  // IGN is only ever HIGH inside this window; always time-limited.
  if (phase == Phase::IGN_HOLD && (millis() - phaseStartMs) >= phaseDuration)
  {
    digitalWrite(IGN_PIN, LOW);
    phase = Phase::IDLE;
    Serial.println("IGN LOW");
  }
}
