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
 */
#ifndef PAD_DIAG_H
#define PAD_DIAG_H

#include <Arduino.h>
#include <stdarg.h>

namespace diag {

/* Where diagnostics go. On a pad node that is the native USB CDC port, the
 * same one the GUI opens -- but the GUI opens it at 115200 and reads bytes,
 * so text must never be interleaved into a binary frame.  Everything here is
 * called from loop() between frames, and the GUI's parser resyncs on 0x7E,
 * so the risk is a dropped frame, never a misinterpreted one. */
static inline void begin() { Serial.begin(115200); }

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
 * Most ESP32-S3 devkits carry a WS2812 on GPIO48, wired through the Arduino
 * core's RGB_BUILTIN.  It is the one channel that works when USB is the thing
 * that is broken.  Colours here are deliberately few:
 *
 *      blue  solid     booting / radio not configured yet
 *      red   solid     RADIO FAULT   (module did not answer -- check wiring)
 *      amber blink 1 Hz  no valid frames from the TX node (link or channel)
 *      green blink      valid frames arriving, pad safe / not armed
 *      purple          igniter energised
 */
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

} /* namespace diag */

#endif /* PAD_DIAG_H */
