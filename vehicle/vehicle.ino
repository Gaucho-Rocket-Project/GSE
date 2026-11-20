/*
  vehicle.ino - ESP-NOW servo / ignition actuator board.

  Upload this sketch to the *actuator* ESP32:
    servo0 signal : GPIO 18  (FILL in live mode)
    servo1 signal : GPIO 5   (SRV in live mode)
    IGN           : GPIO 32, active HIGH

  Two command families are supported:
    1-4    : timed test pulses (serial monitor / test mode)
    10-19  : latched ON/OFF states used by the desktop controller live mode

  Safety interlocks are enforced here as well as in the controller:
    - SRV and IGN cannot turn ON while FILL is open.
    - FILL cannot open while SRV is open or IGN is armed/on.
    - IGN ON is rejected until the arm countdown has elapsed.

  Servos are the DS3225 digital servos from the 52Pi D-0001 wiki:
    4.8-6.8 V supply, 500-2500 us pulse, 50-330 Hz, up to 2.9 A stall each.
    Power them from a separate BEC (>= 6 A peak for two servos) and connect
    the BEC ground to the ESP32 ground. Never power them from the ESP32 5 V pin.

  Arduino IDE:  Tools > Board > ESP32 Dev Module, Serial Monitor at 115200 baud.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>     // esp_wifi_set_ps(), esp_wifi_get_mac()
#include <esp_mac.h>      // esp_read_mac(), ESP_MAC_WIFI_STA
#include <stddef.h>
#include <string.h>

// ---------------------------------------------------------------------------
// USER CONFIGURATION - edit before flashing
// ---------------------------------------------------------------------------
// MAC address printed by the ground sketch at boot (STA MAC).
// With ENFORCE_GROUND_MAC = true, packets from any other board are ignored.
static uint8_t GROUND_MAC[6] = {0xA4, 0xF0, 0x0F, 0x62, 0x0C, 0xB8}; //A4:F0:0F:62:0C:B8
static constexpr bool ENFORCE_GROUND_MAC = true;  // false only for bench testing

static constexpr uint8_t  SERVO0_PIN = 18;
static constexpr uint8_t  SERVO1_PIN = 5;
// UART2 input is newline-delimited printable ASCII; change to match the source.
static constexpr uint32_t TELEMETRY_BAUD = 115200;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 500; // set before flashing
static constexpr int TELEMETRY_RX_PIN = 16;
static constexpr int TELEMETRY_TX_PIN = 17;
static constexpr size_t TELEMETRY_MAX = 200;
static constexpr uint32_t TELEMETRY_MAGIC = 0x314D4C54U; // TLM1
static constexpr uint8_t  IGN_PIN    = 32;        // active HIGH

// LEDC channels (only used by Arduino-ESP32 2.x; 3.x assigns channels itself).
static constexpr uint8_t  SERVO0_CHANNEL = 0;
static constexpr uint8_t  SERVO1_CHANNEL = 1;

static constexpr uint32_t PWM_FREQ_HZ       = 50;   // standard RC servo frame
static constexpr uint8_t  PWM_RESOLUTION_BITS = 16;
static constexpr uint16_t PWM_PERIOD_US     = 1000000UL / PWM_FREQ_HZ;  // 20000 us

static constexpr uint16_t SERVO_MIN_US      = 500;
static constexpr uint16_t SERVO_MAX_US      = 2500;
static constexpr uint16_t SERVO_HOME_US     = 2500; // rest/retracted position
static constexpr uint16_t DEFAULT_OPEN_US   = 2000; // initial latched target; commands must be valid
static constexpr uint16_t MAX_SERVO_HOLD_MS = 5000; // hard cap on test pulse time
static constexpr uint16_t MAX_IGN_ON_MS     = 2000; // hard cap on *test* IGN pulse
static constexpr uint32_t MAX_IGN_ARM_MS    = 60000;
// ---------------------------------------------------------------------------

enum : uint8_t {
  // Timed test actions (reserved = 0).
  CMD_SERVO0 = 1,
  CMD_SERVO1 = 2,
  CMD_BOTH   = 3,
  CMD_IGN    = 4,

  // Latched live-mode states (reserved = 1).
  CMD_SV0_OFF  = 10,
  CMD_SV0_ON   = 11,
  CMD_SV1_OFF  = 12,
  CMD_SV1_ON   = 13,
  CMD_BOTH_OFF = 14,
  CMD_BOTH_ON  = 15,   // rejected: FILL must be closed before SRV opens
  CMD_IGN_OFF  = 16,
  CMD_IGN_ARM  = 17,
  CMD_IGN_ON   = 18,
  CMD_ALL_OFF  = 19
};

// Wire format (must match the ground sketch byte for byte).
static constexpr uint32_t PACKET_MAGIC = 0x314B4352U;  // "RCK1" little endian
static constexpr uint32_t ACK_MAGIC    = 0x314B4341U;  // "ACK1" little endian

struct __attribute__((packed)) CommandPacket {
  uint32_t magic;
  uint32_t token;
  uint8_t  cmd;
  uint8_t  reserved;        // 0 = test pulse, 1 = latched live state
  uint16_t holdMs;          // test hold / IGN arm countdown in ms
  uint16_t servo0PulseUs;   // 500-2500
  uint16_t servo1PulseUs;   // 500-2500
  uint16_t crc16;
};

struct __attribute__((packed)) AckPacket {
  uint32_t magic;
  uint32_t token;
  uint8_t  cmd;
  uint8_t  accepted;
  uint16_t crc16;
};

struct __attribute__((packed)) TelemetryPacket {
  uint32_t magic;
  uint32_t sequence;
  uint32_t uptimeMs;
  uint8_t length;
  char data[TELEMETRY_MAX];
  uint16_t crc16;
};
static_assert(sizeof(TelemetryPacket) <= 250, "ESP-NOW payload too large");
static char telemetryLine[TELEMETRY_MAX + 1] = {};
static char telemetryLatest[TELEMETRY_MAX + 1] = {};
static size_t telemetryLength = 0;
static bool telemetryOverflow = false;
static bool telemetryPending = false;
static uint32_t telemetrySequence = 0;
static uint32_t lastTelemetrySend = 0;

static constexpr size_t CRC_OFFSET    = offsetof(CommandPacket, crc16);
static constexpr size_t ACK_CRC_OFFSET = offsetof(AckPacket, crc16);
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

// ---- Latched live-mode state ----------------------------------------------
static bool     fillOpen        = false;
static bool     srvOpen         = false;
static bool     ignOn           = false;
static bool     ignArmed        = false;
static uint32_t ignArmStartMs   = 0;
static uint32_t ignArmRequiredMs = 0;
static uint16_t fillOpenUs      = DEFAULT_OPEN_US;
static uint16_t srvOpenUs       = DEFAULT_OPEN_US;

static void applyServoPulses()
{
  pwmWrite(SERVO0_PIN, SERVO0_CHANNEL, pulseToDuty(fillOpen ? fillOpenUs : SERVO_HOME_US));
  pwmWrite(SERVO1_PIN, SERVO1_CHANNEL, pulseToDuty(srvOpen ? srvOpenUs : SERVO_HOME_US));
}

static void clearLatched()
{
  fillOpen = false;
  srvOpen = false;
  ignOn = false;
  ignArmed = false;
  digitalWrite(IGN_PIN, LOW);
  applyServoPulses();
}

// ---- Timed test action state machine (non-blocking) -----------------------
enum class Phase : uint8_t { IDLE, SERVO_HOLD, IGN_HOLD };

static Phase    phase          = Phase::IDLE;
static uint32_t phaseStartMs   = 0;
static uint32_t phaseDuration  = 0;
static bool     servo0Active   = false;
static bool     servo1Active   = false;

// Returns true when the command was accepted. Every command gets an ACK.
static bool startAction(const CommandPacket &packet)
{
  if (phase != Phase::IDLE)
  {
    Serial.println("BUSY: command ignored");
    return false;
  }

  switch (packet.cmd)
  {
    // ---- Test mode: timed pulses. Starting a pulse also clears live state.
    case CMD_SERVO0:
    case CMD_SERVO1:
    case CMD_BOTH:
    {
      if (!validPulse(packet.servo0PulseUs) || !validPulse(packet.servo1PulseUs)) return false;
      clearLatched();

      servo0Active = (packet.cmd == CMD_SERVO0 || packet.cmd == CMD_BOTH);
      servo1Active = (packet.cmd == CMD_SERVO1 || packet.cmd == CMD_BOTH);
      applyServoPulses();
      pwmWrite(SERVO0_PIN, SERVO0_CHANNEL,
               pulseToDuty(servo0Active ? packet.servo0PulseUs : SERVO_HOME_US));
      pwmWrite(SERVO1_PIN, SERVO1_CHANNEL,
               pulseToDuty(servo1Active ? packet.servo1PulseUs : SERVO_HOME_US));

      phase = Phase::SERVO_HOLD;
      phaseStartMs = millis();
      phaseDuration = min(packet.holdMs, MAX_SERVO_HOLD_MS);
      Serial.println("Test pulse started");
      return true;
    }

    case CMD_IGN:
    {
      if (fillOpen || srvOpen) return false;
      clearLatched();
      digitalWrite(IGN_PIN, HIGH);
      phase = Phase::IGN_HOLD;
      phaseStartMs = millis();
      phaseDuration = (packet.holdMs == 0) ? 1 : min(packet.holdMs, MAX_IGN_ON_MS);
      Serial.print("Test IGN HIGH for ");
      Serial.print(phaseDuration);
      Serial.println(" ms");
      return true;
    }

    // ---- Live mode: latched states.
    case CMD_SV0_ON:
      if (srvOpen || ignOn || ignArmed) return false;
      if (!validPulse(packet.servo0PulseUs)) return false;
      fillOpen = true;
      fillOpenUs = packet.servo0PulseUs;
      applyServoPulses();
      Serial.println("FILL OPEN");
      return true;

    case CMD_SV0_OFF:
      fillOpen = false;
      applyServoPulses();
      Serial.println("FILL CLOSED");
      return true;

    case CMD_SV1_ON:
      if (fillOpen || ignArmed) return false;
      if (!validPulse(packet.servo1PulseUs)) return false;
      srvOpen = true;
      srvOpenUs = packet.servo1PulseUs;
      applyServoPulses();
      Serial.println("SRV OPEN");
      return true;

    case CMD_SV1_OFF:
      srvOpen = false;
      applyServoPulses();
      Serial.println("SRV CLOSED");
      return true;

    case CMD_BOTH_ON:
      // Interlock: SRV must never open while FILL is open, so a combined ON
      // command is refused by the actuator. Use test command 3 for a pulse.
      Serial.println("BOTH ON refused: FILL must be closed for SRV");
      return false;

    case CMD_BOTH_OFF:
      fillOpen = false;
      srvOpen = false;
      applyServoPulses();
      Serial.println("FILL + SRV CLOSED");
      return true;

    case CMD_IGN_ARM:
      if (fillOpen || srvOpen || ignOn) return false;
      ignOn = false;
      ignArmed = true;
      ignArmStartMs = millis();
      ignArmRequiredMs = min(static_cast<uint32_t>(packet.holdMs), MAX_IGN_ARM_MS);
      digitalWrite(IGN_PIN, LOW);
      Serial.print("IGN arming for ");
      Serial.print(ignArmRequiredMs);
      Serial.println(" ms");
      return true;

    case CMD_IGN_ON:
      if (fillOpen || srvOpen) return false;
      if (!ignArmed || (millis() - ignArmStartMs) < ignArmRequiredMs) return false;
      if (!validPulse(packet.servo1PulseUs)) return false;
      // One validated command applies both outputs; no separate radio command
      // can be lost between ignition and SRV opening.
      srvOpenUs = packet.servo1PulseUs;
      srvOpen = true;
      applyServoPulses();
      ignArmed = false;
      ignOn = true;
      digitalWrite(IGN_PIN, HIGH);
      Serial.println("IGN ON + SRV OPEN (latched)");
      return true;

    case CMD_IGN_OFF:
      ignArmed = false;
      ignOn = false;
      digitalWrite(IGN_PIN, LOW);
      Serial.println("IGN OFF");
      return true;

    case CMD_ALL_OFF:
      clearLatched();
      Serial.println("ALL OFF");
      return true;

    default:
      return false;
  }
}

// ---- Duplicate rejection and ACK ------------------------------------------
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

static void sendAck(const uint8_t *dest, uint32_t token, uint8_t cmd, bool accepted)
{
  if (dest == nullptr) return;
  AckPacket ack{};
  ack.magic = ACK_MAGIC;
  ack.token = token;
  ack.cmd = cmd;
  ack.accepted = accepted ? 1 : 0;
  ack.crc16 = crc16Ccitt(reinterpret_cast<const uint8_t *>(&ack), ACK_CRC_OFFSET);
  esp_now_send(dest, reinterpret_cast<const uint8_t *>(&ack), sizeof(ack));
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

  if (ENFORCE_GROUND_MAC && memcmp(src, GROUND_MAC, 6) != 0) return;

  CommandPacket packet{};
  memcpy(&packet, data, sizeof(packet));

  if (packet.magic != PACKET_MAGIC) return;
  if (packet.crc16 != crc16Ccitt(data, CRC_OFFSET)) return;
  if (packet.token == 0 || tokenAlreadySeen(packet.token)) return;
  if (packet.reserved > 1) return;

  rememberToken(packet.token);
  bool accepted = startAction(packet);
  sendAck(src, packet.token, packet.cmd, accepted);
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(TELEMETRY_BAUD, SERIAL_8N1, TELEMETRY_RX_PIN, TELEMETRY_TX_PIN);
  delay(50);

  // Ignition output is forced LOW before anything else.
  pinMode(IGN_PIN, OUTPUT);
  digitalWrite(IGN_PIN, LOW);

  if (!pwmAttach(SERVO0_PIN, SERVO0_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS) ||
      !pwmAttach(SERVO1_PIN, SERVO1_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS))
  {
    Serial.println("LEDC setup failed");
  }
  applyServoPulses();

  if (ENFORCE_GROUND_MAC && !macConfigured(GROUND_MAC))
  {
    Serial.println("WARNING: GROUND_MAC not set - all commands will be rejected.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);   // ESP-NOW reception is more reliable awake

  uint8_t myMac[6] = {};
  if (readStaMac(myMac)) printMac("Vehicle MAC (copy into ground sketch): ", myMac);
  else Serial.println("Could not read vehicle MAC");

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
    return;
  }

  if (macConfigured(GROUND_MAC))
  {
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, GROUND_MAC, 6);
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
  // Keep only complete, non-empty lines; overlong or non-printable frames are
  // discarded in their entirety rather than silently truncating sensor data.
  while (Serial2.available() > 0)
  {
    char c = static_cast<char>(Serial2.read());
    if (c == '\n')
    {
      if (!telemetryOverflow && telemetryLength > 0)
      {
        telemetryLine[telemetryLength] = 0;
        memcpy(telemetryLatest, telemetryLine, telemetryLength + 1);
        telemetryPending = true;
      }
      telemetryLength = 0;
      telemetryOverflow = false;
    }
    else if (c == '\r') { /* permit CRLF */ }
    else if (c < 32 || c > 126 || telemetryLength >= TELEMETRY_MAX) telemetryOverflow = true;
    else if (!telemetryOverflow) telemetryLine[telemetryLength++] = c;
  }
  if (telemetryPending && millis() - lastTelemetrySend >= TELEMETRY_INTERVAL_MS && macConfigured(GROUND_MAC))
  {
    TelemetryPacket packet{};
    packet.magic = TELEMETRY_MAGIC;
    packet.sequence = ++telemetrySequence;
    packet.uptimeMs = millis();
    packet.length = strlen(telemetryLatest);
    memcpy(packet.data, telemetryLatest, packet.length);
    packet.crc16 = crc16Ccitt(reinterpret_cast<const uint8_t *>(&packet), offsetof(TelemetryPacket, crc16));
    if (esp_now_send(GROUND_MAC, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)) == ESP_OK)
      telemetryPending = false;
    lastTelemetrySend = millis();
  }

  // Return servos to their home pulse once a test hold has elapsed.
  if (phase == Phase::SERVO_HOLD && (millis() - phaseStartMs) >= phaseDuration)
  {
    applyServoPulses();
    servo0Active = false;
    servo1Active = false;
    phase = Phase::IDLE;
    Serial.println("Servos back at home");
  }

  // Test-mode IGN pulse is time-limited; live-mode IGN stays latched until
  // an explicit IGN OFF / ALL OFF command.
  if (phase == Phase::IGN_HOLD && (millis() - phaseStartMs) >= phaseDuration)
  {
    digitalWrite(IGN_PIN, LOW);
    phase = Phase::IDLE;
    Serial.println("IGN LOW");
  }
}
