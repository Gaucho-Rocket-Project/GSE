// -*- mode: c++ -*-
/*
 * Arduino.h  --  a deliberately small stand-in for the Arduino-ESP32 core
 * ---------------------------------------------------------------------------
 * Used only by tests/esp32_syntax_check.sh, which compile-checks the ESP32
 * firmware sources on a workstation where the Espressif toolchain is not
 * installed.  It declares exactly what firmware/src/pad/pad_node.cpp,
 * firmware/src/tx/tx_platform_esp32.cpp and firmware/src/dxlr41.h use, with
 * the same signatures as arduino-esp32 2.x/3.x.
 *
 * It also declares the handful of POSIX / newlib names that the real core's
 * Arduino.h drags in through IPAddress.h and lwIP, so that a firmware symbol
 * which collides with one of them (e.g. a node object called `link`, vs POSIX
 * `int link(...)`) fails HERE instead of in a four-minute cross-compile.  That
 * exact bug shipped once; tests/tripwire/posix_shadow_probe.cpp keeps it
 * honest.
 *
 * It is NOT a simulator and it must never be built as one: the function
 * bodies are empty or return defaults.  Its whole job is to make a typo, a
 * renamed pin or a bad board-variant #if fail in one second instead of in a
 * four-minute cross-compile.
 */
#ifndef ARDUINO_H_SYNTAX_STUB
#define ARDUINO_H_SYNTAX_STUB

#include <stdint.h>
#include <stddef.h>
#include <algorithm>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define ARDUINO_ESP32_SYNTAX_STUB 1

/* The core the stub pretends to be.  Pick one with -D:
 *   default             Arduino-ESP32 3.x API (ledcAttach / rgbLedWrite)
 *   ARDUINO_ESP32_CORE2 Arduino-ESP32 2.x API (ledcSetup / neopixelWrite with
 *                       an RGB_BUILTIN macro, as the S3 devkit variant has)
 */
#if !defined(ARDUINO_ESP32_CORE2)
#  define ESP_ARDUINO_VERSION_MAJOR 3
#  define ESP_ARDUINO_VERSION_MINOR 0
#else
#  define ESP_ARDUINO_VERSION_MAJOR 2
#  define ESP_ARDUINO_VERSION_MINOR 0
#  define RGB_BUILTIN 48
#endif

#define HIGH 0x1
#define LOW  0x0
#define INPUT        0x01
#define OUTPUT       0x03
#define INPUT_PULLUP 0x05

#define SERIAL_8N1 0x800001c
#define LED_BUILTIN 2

typedef uint8_t byte;
typedef bool boolean;

/* ------------------------------------------------------------------- type */
class String {
public:
    String() {}
    String(const char *) {}
    String(int, int = 10) {}          /* covers String(42) and String(42, HEX) */
    String(unsigned) {}
    String(unsigned long) {}
    String(const String &) {}
    String &operator=(const String &) { return *this; }
    String &operator=(const char *) { return *this; }
    String &operator+=(const String &) { return *this; }
    String &operator+=(const char *) { return *this; }
    String &operator+=(char) { return *this; }
    const char *c_str() const { return ""; }
    unsigned length() const { return 0; }
    bool operator==(const char *) const { return false; }
    friend String operator+(const String &, const String &) { return String(); }
    friend String operator+(const String &, const char *) { return String(); }
};

#define F(x) (x)

class Print {
public:
    virtual size_t write(uint8_t) { return 0; }
    size_t write(const uint8_t *, size_t n) { return n; }
    size_t print(const char *) { return 0; }
    size_t print(char) { return 0; }
    size_t print(int) { return 0; }
    size_t print(unsigned) { return 0; }
    size_t println(const char *) { return 0; }
    size_t printf(const char *, ...) { return 0; }
};

class Stream : public Print {
public:
    int available() { return 0; }
    int read() { return -1; }
    int peek() { return -1; }
    void flush() {}
    void setTimeout(unsigned long) {}
};

class HardwareSerial : public Stream {
public:
    explicit HardwareSerial(int) {}
    void begin(unsigned long) {}
    void begin(unsigned long, uint32_t) {}
    void begin(unsigned long, uint32_t, int8_t, int8_t) {}
    void end() {}
    void setRx(int8_t) {}
    void setTx(int8_t) {}
    size_t write(const uint8_t *, size_t n) { return n; }
    size_t write(uint8_t) { return 1; }
    operator bool() const { return true; }
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

/* ------------------------------------------------------------------ time/GPIO */
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int);
void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int  digitalRead(uint8_t);
int  analogRead(uint8_t);
void analogWrite(uint8_t, int);
/* arduino-esp32's Arduino.h pulls in <algorithm> and re-exports these, which
 * is why the firmware may write min<uint32_t>(...) as a template. */
using std::min;
using std::max;
#ifndef constrain
#  define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

/* --------------------------------------------------------------------- LEDC */
#if ESP_ARDUINO_VERSION_MAJOR >= 3
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
bool ledcWrite(uint8_t pin, uint32_t duty);
bool ledcDetach(uint8_t pin);
void rgbLedWrite(uint8_t pin, uint8_t r, uint8_t g, uint8_t b);
#else
void ledcSetup(uint8_t chan, double freq, uint8_t resolution);
void ledcAttachPin(uint8_t pin, uint8_t chan);
void ledcWrite(uint8_t chan, uint32_t duty);
void neopixelWrite(uint8_t pin, uint8_t r, uint8_t g, uint8_t b);
#endif

/* ------------------------------------------------------------------ watchdog */
typedef struct {
    uint32_t timeout_ms;
    uint32_t idle_core_mask;
    bool     trigger_panic;
} esp_task_wdt_config_t;

typedef enum { ESP_OK_S = 0, ESP_ERR_INVALID_STATE_S = 0x103 } esp_err_t;
esp_err_t esp_task_wdt_init(uint32_t timeout_s, bool panic);
esp_err_t esp_task_wdt_reconfigure(const esp_task_wdt_config_t *);
esp_err_t esp_task_wdt_add(void *);
esp_err_t esp_task_wdt_reset(void);

/* A couple of ESP32-only constants a board variant might reference, so that a
 * build which accidentally uses one on the wrong board is caught by the
 * undefined-identifier rule rather than by an exception on the bench. */
#define GPIO_NUM_NC (-1)

/* --------------------------------------------------- POSIX / newlib names
 * The REAL arduino-esp32 Arduino.h is not a small header: it includes
 * IPAddress.h, which includes lwIP, which includes <unistd.h>, <string.h> and
 * friends.  Those drag a pile of global-scope POSIX declarations into every
 * firmware translation unit, and any of them can collide with a firmware
 * symbol -- which is exactly how `static GseLink link;` failed to build on a
 * real ESP-32S while this stub happily accepted it (POSIX `link()` was not
 * declared here).
 *
 * So the few POSIX names that are (a) actually reachable from the real
 * Arduino.h and (b) plausible variable names in this codebase are declared
 * here too, with the same signatures.  If the firmware ever shadows one of
 * them again, the host check fails in a second instead of on the bench.  This
 * list is not exhaustive -- it is a tripwire, not a libc.  Only names whose
 * signatures do not clash with the host's own headers belong here (index(),
 * select(), gettimeofday() are already visible to the host compiler). */
int   link(const char *path1, const char *path2);      /* <unistd.h>        */
int   unlink(const char *path);                        /* <unistd.h>        */
int   read(int fd, void *buf, unsigned int n);         /* <unistd.h>        */
int   write(int fd, const void *buf, unsigned int n);  /* <unistd.h>        */
int   close(int fd);                                   /* <unistd.h>        */
int   open(const char *path, int flags, ...);          /* <fcntl.h>         */
int   ioctl(int fd, int request, ...);                 /* <sys/ioctl.h>     */
void  sync(void);                                      /* <unistd.h>        */
int   pipe(int fds[2]);                                /* <unistd.h>        */


/* ------------------------------- FreeRTOS critical section (crit_enter/exit)
 * real Arduino.h reaches these through freertos/FreeRTOS.h.  The BLE transport
 * guards its receive ring with one, so the stub needs the same names. */
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
void portENTER_CRITICAL(portMUX_TYPE *mux);
void portEXIT_CRITICAL(portMUX_TYPE *mux);

#endif /* ARDUINO_H_SYNTAX_STUB */
