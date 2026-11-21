// Exercise the actual vehicle sketch against lightweight host ESP32 stubs.
#include <cassert>
#define ESP_ARDUINO_VERSION_MAJOR 3
#include "../../vehicle/vehicle.ino"
uint32_t fakeMillis = 0;
uint32_t duties[64] = {};
int pinValues[64] = {};
unsigned risingEdges[64] = {};
FakeSerial Serial, Serial2;
FakeWiFi WiFi;

static CommandPacket command(uint8_t cmd) {
  CommandPacket p{};
  p.cmd = cmd;
  p.servo0PulseUs = 1200;
  p.servo1PulseUs = 1800;
  p.holdMs = 10;
  return p;
}
static bool run(uint8_t cmd) { return startAction(command(cmd)); }
static void armAndFire() {
  assert(run(CMD_IGN_ARM));
  assert(!run(CMD_IGN_ON)); // not before countdown
  fakeMillis += 10;
  assert(run(CMD_IGN_ON));
  assert(pinValues[IGN_PIN] == HIGH);
  assert(srvOpen);
  assert(duties[SERVO1_PIN] == pulseToDuty(1800));
}
int main() {
  setup();
  assert(pinValues[IGN_PIN] == LOW);
  assert(run(CMD_SV0_ON));
  assert(!run(CMD_IGN_ARM));
  assert(!run(CMD_IGN_ON));
  assert(run(CMD_SV0_OFF));
  armAndFire();
  unsigned edges = risingEdges[IGN_PIN];
  assert(!run(CMD_IGN_ON));
  assert(!run(CMD_SV1_OFF));
  assert(!run(CMD_BOTH_OFF));
  assert(!run(CMD_SERVO1));
  fakeMillis += LIVE_IGN_PULSE_MS - 1;
  loop();
  assert(pinValues[IGN_PIN] == HIGH);
  assert(duties[SERVO1_PIN] == pulseToDuty(1800));
  ++fakeMillis;
  loop();
  assert(pinValues[IGN_PIN] == LOW);
  assert(srvOpen && ignOn);
  assert(duties[SERVO1_PIN] == pulseToDuty(1800));
  fakeMillis += 10000;
  loop();
  assert(srvOpen && duties[SERVO1_PIN] == pulseToDuty(1800));
  assert(risingEdges[IGN_PIN] == edges);
  assert(run(CMD_IGN_OFF));
  assert(!srvOpen && !ignOn && pinValues[IGN_PIN] == LOW);
  assert(duties[SERVO1_PIN] == pulseToDuty(SERVO_HOME_US));
  armAndFire();
  assert(run(CMD_IGN_OFF)); // reset while pulse is still high
  assert(pinValues[IGN_PIN] == LOW && !srvOpen);
  fakeMillis += LIVE_IGN_PULSE_MS;
  loop();
  assert(pinValues[IGN_PIN] == LOW && !srvOpen);
  armAndFire();
  assert(run(CMD_ALL_OFF));
  assert(pinValues[IGN_PIN] == LOW && !srvOpen);
  assert(duties[SERVO1_PIN] == pulseToDuty(SERVO_HOME_US));
  // millis() rollover is handled by unsigned subtraction.
  fakeMillis = 0xFFFFFFF0u;
  armAndFire();
  fakeMillis += LIVE_IGN_PULSE_MS;
  loop();
  assert(pinValues[IGN_PIN] == LOW && srvOpen);
  assert(run(CMD_IGN_OFF));
  assert(run(CMD_IGN));
  assert(pinValues[IGN_PIN] == HIGH);
  assert(run(CMD_ALL_OFF)); // safety command preempts a test pulse
  assert(pinValues[IGN_PIN] == LOW && phase == Phase::IDLE);

  // Real receive path: retransmissions of the same token must not retrigger
  // or extend the pulse, even after the pulse has ended.
  assert(run(CMD_IGN_ARM));
  fakeMillis += 10;
  CommandPacket packet = command(CMD_IGN_ON);
  packet.magic = PACKET_MAGIC;
  packet.token = 1234;
  packet.crc16 = crc16Ccitt(reinterpret_cast<const uint8_t *>(&packet), CRC_OFFSET);
  esp_now_recv_info_t info{GROUND_MAC};
  unsigned before = risingEdges[IGN_PIN];
  onDataRecv(&info, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  assert(risingEdges[IGN_PIN] == before + 1);
  fakeMillis += LIVE_IGN_PULSE_MS / 2;
  onDataRecv(&info, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  fakeMillis += LIVE_IGN_PULSE_MS / 2;
  loop();
  assert(pinValues[IGN_PIN] == LOW && srvOpen);
  onDataRecv(&info, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  assert(pinValues[IGN_PIN] == LOW && risingEdges[IGN_PIN] == before + 1);
  assert(run(CMD_IGN_OFF));
}
