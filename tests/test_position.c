/*
 * Host-side unit test for the position parser (no ESP32 hardware needed).
 *
 *   make -C tests          # builds and runs
 *   or: gcc -I../firmware/main ../firmware/main/position.c test_position.c -o t && ./t
 */
#include <stdio.h>
#include <string.h>

#include "position.h"

#define MIN_US 500
#define MAX_US 2500

/* Same mapping the firmware uses: 0 deg -> 500 us, 180 deg -> 2500 us */
static int angle_to_us(int deg)
{
    if (deg < 0) {
        deg = 0;
    } else if (deg > 180) {
        deg = 180;
    }
    return MIN_US + (deg * (MAX_US - MIN_US)) / 180;
}

static int failures;

static void expect(const char *in, int want)
{
    int got = position_parse_pulse_us(in, MIN_US, MAX_US, angle_to_us);
    int ok = (got == want);
    if (!ok) {
        failures++;
    }
    printf("  %-10s -> %5d us   (want %5d)  %s\n",
           (in && *in) ? in : "<empty>", got, want, ok ? "ok" : "** FAIL **");
}

int main(void)
{
    printf("position parser unit tests\n");
    printf("--------------------------\n");

    /* angles */
    expect("0", 500);
    expect("90", 1500);
    expect("180", 2500);
    expect("90deg", 1500);
    expect("45deg", 1000);
    expect("200deg", 2500);        /* clamped */
    expect("\xc2\xb0", -1);        /* lone degree sign is not a number */

    /* a bare small integer is read as an angle, a decimal as milliseconds */
    expect("1", 511);
    expect("1.5", 1500);
    expect("1.5ms", 1500);
    expect("2ms", 2000);
    expect("2.5ms", 2500);         /* clamped to max */
    expect("0.5ms", 500);
    expect("0.1ms", 500);          /* clamped to min */

    /* microseconds */
    expect("1500", 1500);
    expect("1000us", 1000);
    expect("1.8ms", 1800);
    expect("9999us", 2500);        /* clamped */
    expect("20us", 500);           /* clamped */

    /* rejected input */
    expect("abc", -1);
    expect("90xyz", -1);
    expect("", -1);
    expect(NULL, -1);
    expect("deg", -1);

    /* suffix detection used for CLI disambiguation */
    printf("\nsuffix detection\n-----------------\n");
    const char *with_units[] = { "90deg", "1.5ms", "1500us", "2d" };
    const char *no_units[]   = { "90", "1500", "1.5", "servo1", "on", "both" };
    for (size_t i = 0; i < sizeof(with_units) / sizeof(*with_units); i++) {
        int ok = position_has_unit_suffix(with_units[i]) == true;
        if (!ok) {
            failures++;
        }
        printf("  %-10s has unit: %s\n", with_units[i], ok ? "ok" : "** FAIL **");
    }
    for (size_t i = 0; i < sizeof(no_units) / sizeof(*no_units); i++) {
        int ok = position_has_unit_suffix(no_units[i]) == false;
        if (!ok) {
            failures++;
        }
        printf("  %-10s no unit : %s\n", no_units[i], ok ? "ok" : "** FAIL **");
    }

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
