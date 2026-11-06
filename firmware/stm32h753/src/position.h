/*
 * position.h - parse user-typed servo position tokens
 *
 * Deliberately free of any hardware or libc dependency so the exact same code
 * can be unit tested on a PC (see tests/test_position_stm32.c).
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** True if the token ends in a unit suffix (deg / d / ms / us / °). */
bool position_has_unit_suffix(const char *tok);

/**
 * @brief Parse a position token into a pulse width in microseconds.
 *
 * Accepted forms
 *   90 / 90deg / 90°   -> angle, converted with @p angle_to_us
 *   1.5 / 1.5ms        -> milliseconds
 *   1500 / 1500us      -> microseconds
 *
 * A bare number is interpreted by magnitude: <= 270 is an angle, > 270 is a
 * pulse width in microseconds, and anything with a decimal point is ms.
 *
 * @param min_us,max_us  result is clamped into this inclusive range
 * @param angle_to_us    degrees -> microseconds converter (may be NULL)
 * @return pulse width in us, or -1 if the token is not a position
 */
int position_parse_pulse_us(const char *tok, int min_us, int max_us,
                            int (*angle_to_us)(int));

#ifdef __cplusplus
}
#endif
