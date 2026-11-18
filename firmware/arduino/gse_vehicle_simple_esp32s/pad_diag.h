/*
 * pad_diag.h  --  what the pad node says out loud
 * ---------------------------------------------------------------------------
 * The pad node used to print nothing at all.  That is how a bricked-radio pad
 * looks exactly like a healthy-but-unlinked pad: silence, for ever.  This
 * header gives it a voice on the same USB port the GUI uses, in the same
 * "LOG ..." text form the TX node already speaks, so the same terminal and the
 * same GUI console show it.
 *
 * What gets printed:
 *   boot banner            one line, always, before any radio work
 *   RADIO OK / RADIO FAULT the provisioning result, with the failing command
 *   RADIO NOW              the module's own AT+HELP dump, verbatim
 *   per-5 s heartbeat      state, valid/invalid frame counts, age
 *   frame counter at 1 Hz  so a dead link vs a wrong channel is obvious in 2 s
 *
 * Nothing here can hold a valve open: it only writes to Serial.
 *
 * Works on both board variants.  The laptop port is whatever config.h says
 * (GSE_HOST_BAUD): native USB-CDC on an ESP32-S3, the CP2102/CH340 bridge on
 * UART0 for an "ESP-32S Development Board".  The status indication is a WS2812
 * on the S3 devkits and the plain on-board blue LED on an ESP-32S, where
 * "colour" collapses to a four-state blink pattern.
 */
#ifndef PAD_DIAG_H
#define PAD_DIAG_H

#include <Arduino.h>
#include <stdarg.h>

#include "config.h"

namespace diag {

/* Where diagnostics go. On a pad node that is the port the GUI opens -- on an
 * ESP32-S3 the native USB CDC port, on an ESP-32S the USB-UART bridge -- but
 * the GUI opens it at 115200 and reads bytes, so text must never be interleaved
 * into a binary frame.  Everything here is called from loop() between frames,
 * and the GUI's parser resyncs on 0x7E, so the risk is a dropped frame, never
 * a misinterpreted one. */
static inline void begin() { Serial.begin(GSE_HOST_BAUD); }

static inline void line(const char *fmt, ...)
{
    char buf[224];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    Serial.printf("LOG %s\r\n", buf);
}

/* Print a multi-line blob (e.g. the module's AT+HELP banner) one CRLF per
 * line, each prefixed, so the GUI console keeps them separate. */
static inline void blob(const char *tag, const char *text)
{
    char buf[64];
    snprintf(buf, sizeof buf, "LOG %s ", tag);
    Serial.print(buf);
    for (const char *p = text; *p; ++p) {
        if (*p == '\n') Serial.print("\r\nLOG | ");
        else if (*p != '\r') Serial.write((uint8_t)*p);
    }
    Serial.print("\r\n");
}

/* ---------------------------------------------------------------- status LED
 * Two kinds of indicator, chosen at compile time by config.h:
 *
 * ESP32-S3 devkits -- WS2812 on GPIO48 through the Arduino core's RGB_BUILTIN.
 *   Colours are deliberately few:
 *
 *      blue  solid       booting / radio not configured yet
 *      red   solid       RADIO FAULT   (module did not answer -- check wiring)
 *      amber blink 1 Hz  no valid frames from the TX node (link or channel)
 *      green blink       valid frames arriving, pad safe / not armed
 *      purple            igniter energised
 *
 * ESP-32S -- one blue LED on GPIO2, so colour is impossible and the blink
 *   pattern carries the state instead:
 *
 *      5 Hz blink        RADIO FAULT
 *      2 Hz blink        no valid frames (link or channel)
 *      1 Hz blink        valid frames, pad safe / not armed
 *      10 Hz fast blink  igniter energised (cannot be missed)
 *      solid on          booting, before the first radio verdict
 *
 * A blink rate is read at a glance from the far end of a firing line, which is
 * the only thing this LED has to do on either board.                        */
#if GSE_HAS_RGB_LED

static inline void led(uint8_t r, uint8_t g, uint8_t b)
{
#if defined(RGB_BUILTIN)
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    rgbLedWrite(RGB_BUILTIN, r, g, b);          /* core 3.x                */
  #else
    neopixelWrite(RGB_BUILTIN, r, g, b);        /* core 2.x                */
  #endif
#else
    (void)r; (void)g; (void)b;
#endif
}

#else  /* plain single-colour LED */

static inline void led(uint8_t r, uint8_t g, uint8_t b)
{
    (void)r; (void)g; (void)b;
}

/* brightness 0..255, and a half-period in ms; 0 = solid on.
 * Half-periods rather than "blink rate" so the call sites read the same as the
 * colour version: "led_on_blink(255, 0)" == 1 Hz, "led_on_blink(255, 100)"
 * == 5 Hz.                                                                */
static inline void led_simple(uint8_t level, uint16_t half_period_ms)
{
#if PIN_STATUS_LED >= 0
    /* "on" as a logical state; the electrical level depends on how the board
     * wires the LED (most ESP-32S devkits: active HIGH on GPIO2).           */
    bool on;
    if (half_period_ms == 0) {
        on = level != 0;
    } else {
        /* millis()-based so it does not block and does not drift with loop() */
        on = (((millis() / half_period_ms) & 1u) == 0) && level != 0;
    }
    digitalWrite(PIN_STATUS_LED, (on != (GSE_STATUS_LED_ACTIVE_LOW != 0)) ? HIGH : LOW);
#else
    (void)level; (void)half_period_ms;
#endif
}

#endif /* GSE_HAS_RGB_LED */

/* Called once from setup(); a no-op on boards whose indicator needs no pin
 * setup (the WS2812 is driven by the core, and PIN_STATUS_LED is -1 there). */
static inline void led_pin_begin()
{
#if !GSE_HAS_RGB_LED && PIN_STATUS_LED >= 0
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, GSE_STATUS_LED_ACTIVE_LOW ? HIGH : LOW);  /* off */
#endif
}

} /* namespace diag */

#endif /* PAD_DIAG_H */
