#pragma once
#include <cstdint>
#include <cstddef>
#include <algorithm>
using std::min;
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define HEX 16
#define SERIAL_8N1 0
extern uint32_t fakeMillis;
extern uint32_t duties[64];
extern int pinValues[64];
extern unsigned risingEdges[64];
inline uint32_t millis() { return fakeMillis; }
inline void delay(unsigned long ms) { fakeMillis += ms; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int value) {
  if (value == HIGH && pinValues[pin] != HIGH) ++risingEdges[pin];
  pinValues[pin] = value;
}
inline bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
inline void ledcWrite(uint8_t pin, uint32_t duty) { duties[pin] = duty; }
struct FakeSerial {
  void begin(unsigned, ...) {}
  int available() { return 0; }
  int read() { return -1; }
  template<class T> void print(T) {}
  template<class T> void print(T, int) {}
  template<class T> void println(T) {}
  template<class T> void println(T, int) {}
  void println() {}
};
extern FakeSerial Serial, Serial2;
