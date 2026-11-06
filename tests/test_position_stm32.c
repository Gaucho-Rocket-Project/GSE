/*
 * test_position_stm32.c - host-side unit tests for the position parser used by
 * the NUCLEO-H753ZI servo firmware.
 *
 * The parser (firmware/stm32h753/src/position.c) is deliberately free of any
 * hardware dependency, so the exact same source that runs on the STM32 can be
 * exercised on a PC:
 *
 *     make -C tests test-stm32
 */
#include <stdio.h>
#include <string.h>

#include "position.h"

static int failures;
static int checks;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        checks++;                                                           \
        if (!(cond)) {                                                      \
            failures++;                                                     \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);                    \
            printf(__VA_ARGS__);                                            \
            printf("\n");                                                   \
        }                                                                   \
    } while (0)

/* Same maths as servo_angle_to_pulse_us(), 500..2500 us over 180 deg. */
static int angle_to_us_180(int deg)
{
    if (deg < 0) {
        deg = 0;
    } else if (deg > 180) {
        deg = 180;
    }
    return 500 + (deg * 2000) / 180;
}

/* 500..2500 us over 270 deg (the DS3225 270 variant). */
static int angle_to_us_270(int deg)
{
    if (deg < 0) {
        deg = 0;
    } else if (deg > 270) {
        deg = 270;
    }
    return 500 + (deg * 2000) / 270;
}

static void test_units(void)
{
    CHECK(position_has_unit_suffix("1.5ms"), "1.5ms should have a unit");
    CHECK(position_has_unit_suffix("1500us"), "1500us should have a unit");
    CHECK(position_has_unit_suffix("90deg"), "90deg should have a unit");
    CHECK(!position_has_unit_suffix("90"), "90 should not have a unit");
    CHECK(!position_has_unit_suffix("servo1"), "servo1 should not have a unit");
    CHECK(!position_has_unit_suffix(""), "empty should not have a unit");
    CHECK(!position_has_unit_suffix(NULL), "NULL should not have a unit");
}

static void test_angles(void)
{
    CHECK(position_parse_pulse_us("0", 500, 2500, angle_to_us_180) == 500,
          "0 deg -> 500 us");
    CHECK(position_parse_pulse_us("90", 500, 2500, angle_to_us_180) == 1500,
          "90 deg -> 1500 us");
    CHECK(position_parse_pulse_us("180", 500, 2500, angle_to_us_180) == 2500,
          "180 deg -> 2500 us");
    CHECK(position_parse_pulse_us("45deg", 500, 2500, angle_to_us_180) == 1000,
          "45deg -> 1000 us");
    CHECK(position_parse_pulse_us("180d", 500, 2500, angle_to_us_180) == 2500,
          "180d -> 2500 us");
    CHECK(position_parse_pulse_us("180degrees", 500, 2500, angle_to_us_180) == 2500,
          "180degrees -> 2500 us");
    CHECK(position_parse_pulse_us("\xc2\xb0", 500, 2500, angle_to_us_180) == -1,
          "bare degree sign is not a position");
    CHECK(position_parse_pulse_us("135deg", 500, 2500, angle_to_us_270) == 1500,
          "135 deg on a 270 servo -> 1500 us");
    CHECK(position_parse_pulse_us("90", 500, 2500, angle_to_us_270) == 1166,
          "90 deg on a 270 servo -> 1166 us");
}

static void test_milliseconds(void)
{
    CHECK(position_parse_pulse_us("1.5ms", 500, 2500, angle_to_us_180) == 1500,
          "1.5ms -> 1500 us");
    CHECK(position_parse_pulse_us("1.5", 500, 2500, angle_to_us_180) == 1500,
          "1.5 -> 1500 us (decimal implies ms)");
    CHECK(position_parse_pulse_us("1.2ms", 500, 2500, angle_to_us_180) == 1200,
          "1.2ms -> 1200 us");
    CHECK(position_parse_pulse_us("0.5", 500, 2500, angle_to_us_180) == 500,
          "0.5 -> 500 us");
    CHECK(position_parse_pulse_us("2.5ms", 500, 2500, angle_to_us_180) == 2500,
          "2.5ms -> 2500 us");
    CHECK(position_parse_pulse_us("2.0ms", 500, 2500, angle_to_us_180) == 2000,
          "2.0ms -> 2000 us");
}

static void test_microseconds(void)
{
    CHECK(position_parse_pulse_us("1500", 500, 2500, angle_to_us_180) == 1500,
          "1500 -> 1500 us");
    CHECK(position_parse_pulse_us("1500us", 500, 2500, angle_to_us_180) == 1500,
          "1500us -> 1500 us");
    CHECK(position_parse_pulse_us("750us", 500, 2500, angle_to_us_180) == 750,
          "750us -> 750 us");
    CHECK(position_parse_pulse_us("2000us", 500, 2500, angle_to_us_180) == 2000,
          "2000us -> 2000 us");
}

static void test_clamping(void)
{
    CHECK(position_parse_pulse_us("100us", 500, 2500, angle_to_us_180) == 500,
          "too-short pulse clamps up");
    CHECK(position_parse_pulse_us("9999us", 500, 2500, angle_to_us_180) == 2500,
          "too-long pulse clamps down");
    CHECK(position_parse_pulse_us("-90", 500, 2500, angle_to_us_180) == 500,
          "negative clamps to the low end");
    CHECK(position_parse_pulse_us("300deg", 500, 2500, angle_to_us_180) == 2500,
          "angle beyond travel clamps");
}

static void test_garbage(void)
{
    CHECK(position_parse_pulse_us("", 500, 2500, angle_to_us_180) == -1, "empty");
    CHECK(position_parse_pulse_us(NULL, 500, 2500, angle_to_us_180) == -1, "NULL");
    CHECK(position_parse_pulse_us("abc", 500, 2500, angle_to_us_180) == -1, "abc");
    CHECK(position_parse_pulse_us("90xyz", 500, 2500, angle_to_us_180) == -1, "tail");
    CHECK(position_parse_pulse_us("servo1", 500, 2500, angle_to_us_180) == -1, "servo");
    CHECK(position_parse_pulse_us("ms", 500, 2500, angle_to_us_180) == -1, "unit only");
    CHECK(position_parse_pulse_us("1.5.5ms", 500, 2500, angle_to_us_180) == -1, "double dot");
}

static void test_whitespace_and_null_converter(void)
{
    CHECK(position_parse_pulse_us(" 90 ", 500, 2500, angle_to_us_180) == 1500,
          "leading/trailing spaces are tolerated");
    CHECK(position_parse_pulse_us(" 1500 us ", 500, 2500, angle_to_us_180) == 1500,
          "space before the unit is tolerated");
    CHECK(position_parse_pulse_us("90", 0, 4000, NULL) == 90,
          "NULL converter returns the raw angle");
}

int main(void)
{
    test_units();
    test_angles();
    test_milliseconds();
    test_microseconds();
    test_clamping();
    test_garbage();
    test_whitespace_and_null_converter();

    printf("%d checks, %d failure(s)\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
