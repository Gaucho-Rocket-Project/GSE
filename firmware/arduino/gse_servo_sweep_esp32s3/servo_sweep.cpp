/*
 * servo_sweep.cpp  --  bench test: sweep the pad's servos through 180 deg
 * ---------------------------------------------------------------------------
 * NOT part of the pad firmware.  PlatformIO never builds this folder; the
 * Arduino sketch firmware/arduino/gse_servo_sweep_esp32s3/ is generated from it
 * by tools/make_arduino_sketches.py.  Flash it, test, then flash the real pad
 * firmware back.
 *
 * Based on the ESP32Servo "Sweep" example, changed for this pad:
 *   - every fitted servo -- FILL, both VEHICLE servos, PURGE -- on the pad
 *     firmware's own pins (PIN_SERVO_* from config.h; a pin of -1 is skipped),
 *     so a pin change there is picked up here on the next regenerate;
 *   - the DS3225's 180-degree range: 500..2500 us, the same limits the pad
 *     firmware clamps to.  Angles here are 0..180 with 90 = 1500 us = the
 *     pad's neutral, so the pad's "travel t" positions are 90 + t (OPEN) and
 *     90 - t (CLOSED) on this scale;
 *   - each servo's direction and trim from config.h (GSE_SERVO_*_REV /
 *     _TRIM_US) are applied exactly as the pad firmware applies them, so a
 *     mirrored vehicle servo sweeps mirrored here too;
 *   - the igniter relay pin is held LOW and the keep-alive heartbeat is never
 *     toggled, so the relay chain stays dead for the whole test;
 *   - the angle and each servo's pulse width are printed every 15 deg.
 *
 * The full 0..180 sweep drives each servo to both ends of its travel.  If a
 * servo is coupled to a valve whose handle cannot turn that far, narrow
 * SWEEP_MIN_DEG / SWEEP_MAX_DEG first or uncouple the linkage.
 */
#include <Arduino.h>
#include <ESP32Servo.h>

#include "config.h"

/* ------------------------------------------------------------ test settings */
static const int SWEEP_MIN_DEG = 0;      /* 0   = 500 us  = pad CLOSED at travel 90 */
static const int SWEEP_MAX_DEG = 180;    /* 180 = 2500 us = pad OPEN   at travel 90 */
static const int STEP_DELAY_MS = 15;     /* per 1-degree step, as in the example   */

/* ------------------------------------------------------------------ servos */
struct TestServo { const char *name; int pin; bool reversed; int trimUs; };
static const TestServo kTest[] = {
    { "FILL", PIN_SERVO_FILL, GSE_SERVO_FILL_REV != 0, GSE_SERVO_FILL_TRIM_US },
    { "VEH",  PIN_SERVO_VEH,  GSE_SERVO_VEH_REV  != 0, GSE_SERVO_VEH_TRIM_US  },
    { "VEH2", PIN_SERVO_VEH2, GSE_SERVO_VEH2_REV != 0, GSE_SERVO_VEH2_TRIM_US },
    { "PRG",  PIN_SERVO_PRG,  GSE_SERVO_PRG_REV  != 0, GSE_SERVO_PRG_TRIM_US  },
};
static const int N = sizeof kTest / sizeof kTest[0];
static Servo servos[N];

/* deg 0..180 -> pulse, the pad firmware's way: 500 us per 45 deg around
 * 1500 us, mirrored if reversed, plus trim, clamped to 500..2500 us. */
static int pulseFor(const TestServo &s, int deg)
{
    int off = SERVO_US_PER_45DEG * (deg - 90) / 45;
    if (s.reversed) off = -off;
    return constrain(SERVO_NEUTRAL_US + s.trimUs + off, SERVO_MIN_US, SERVO_MAX_US);
}

static void writeAll(int deg)
{
    for (int i = 0; i < N; ++i)
        if (kTest[i].pin >= 0) servos[i].writeMicroseconds(pulseFor(kTest[i], deg));
}

static void report(int deg)
{
    if (deg % 15 != 0 && deg != SWEEP_MIN_DEG && deg != SWEEP_MAX_DEG) return;
    Serial.printf("deg %3d (%+3d)", deg, deg - 90);
    for (int i = 0; i < N; ++i)
        if (kTest[i].pin >= 0) Serial.printf("  %s %4d us", kTest[i].name, pulseFor(kTest[i], deg));
    Serial.println();
}

void setup()
{
    /* Relay chain off before anything else, and kept off. */
    pinMode(PIN_RELAY_IGN, OUTPUT);
    digitalWrite(PIN_RELAY_IGN, LOW);
    pinMode(PIN_HB, OUTPUT);
    digitalWrite(PIN_HB, LOW);

    Serial.begin(GSE_HOST_BAUD);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    for (int i = 0; i < N; ++i) {
        if (kTest[i].pin < 0) continue;                     /* not fitted     */
        servos[i].setPeriodHertz(50);                       /* standard 50 Hz */
        servos[i].attach(kTest[i].pin, SERVO_MIN_US, SERVO_MAX_US);
        servos[i].writeMicroseconds(pulseFor(kTest[i], 90));  /* centre      */
    }

    delay(500);
    Serial.printf("servo sweep test: %d..%d deg, igniter GPIO%d held LOW, heartbeat off\n",
                  SWEEP_MIN_DEG, SWEEP_MAX_DEG, PIN_RELAY_IGN);
    for (int i = 0; i < N; ++i) {
        if (kTest[i].pin < 0) Serial.printf("  %-4s not fitted\n", kTest[i].name);
        else Serial.printf("  %-4s GPIO%-2d %s trim %+d us\n", kTest[i].name, kTest[i].pin,
                           kTest[i].reversed ? "REVERSED" : "normal  ", kTest[i].trimUs);
    }
}

void loop()
{
    for (int deg = SWEEP_MIN_DEG; deg <= SWEEP_MAX_DEG; ++deg) {
        writeAll(deg);
        report(deg);
        delay(STEP_DELAY_MS);
    }
    for (int deg = SWEEP_MAX_DEG; deg >= SWEEP_MIN_DEG; --deg) {
        writeAll(deg);
        report(deg);
        delay(STEP_DELAY_MS);
    }
}
