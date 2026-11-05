/*
 * position.c - parse a user-typed position token into a servo pulse width
 *
 * Kept free of ESP-IDF dependencies so it can be unit-tested on a PC.
 */
#include "position.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

bool position_has_unit_suffix(const char *tok)
{
    if (tok == NULL || *tok == '\0') {
        return false;
    }
    /* A unit suffix only counts when it follows a number: "1.5ms" yes,
     * "servo1" no (letters before any digit). */
    while (*tok != '\0' &&
           (isdigit((unsigned char)*tok) || *tok == '.' || *tok == '-' || *tok == '+')) {
        tok++;
    }
    if (*tok == '\0') {
        return false;                  /* plain number, no unit */
    }
    return strcmp(tok, "deg") == 0 || strcmp(tok, "d") == 0 ||
           strcmp(tok, "degree") == 0 || strcmp(tok, "degrees") == 0 ||
           strcmp(tok, "ms") == 0 || strcmp(tok, "msec") == 0 ||
           strcmp(tok, "us") == 0 || strcmp(tok, "usec") == 0 ||
           strcmp(tok, "\xc2\xb0") == 0;
}

int position_parse_pulse_us(const char *tok, int min_us, int max_us,
                            int (*angle_to_us)(int))
{
    char   buf[32];
    size_t len;

    if (tok == NULL || *tok == '\0') {
        return -1;
    }
    len = strlen(tok);
    if (len >= sizeof(buf)) {
        return -1;
    }
    memcpy(buf, tok, len + 1);

    /* Split "<number><suffix>" */
    char *suffix = buf;
    while (*suffix != '\0' &&
           (isdigit((unsigned char)*suffix) || *suffix == '.' || *suffix == '-' || *suffix == '+')) {
        suffix++;
    }
    if (suffix == buf) {
        return -1;                      /* no digits at all */
    }

    bool is_deg = false;
    bool is_ms  = false;
    bool is_us  = false;

    if (strcmp(suffix, "deg") == 0 || strcmp(suffix, "d") == 0 ||
        strcmp(suffix, "degree") == 0 || strcmp(suffix, "degrees") == 0 ||
        strcmp(suffix, "\xc2\xb0") == 0) {          /* UTF-8 "°" */
        is_deg = true;
    } else if (strcmp(suffix, "ms") == 0 || strcmp(suffix, "msec") == 0) {
        is_ms = true;
    } else if (strcmp(suffix, "us") == 0 || strcmp(suffix, "usec") == 0) {
        is_us = true;
    } else if (*suffix != '\0') {
        return -1;                      /* trailing garbage, e.g. "90xyz" */
    }

    char *end = NULL;
    float value = strtof(buf, &end);
    if (end == buf) {
        return -1;
    }

    if (!is_deg && !is_ms && !is_us) {
        /* No unit given - infer it from the magnitude:
         *   "1.5"   -> 1.5 ms      (has a decimal point)
         *   "1500"  -> 1500 us     (large integer)
         *   "90"    -> 90 degrees  (0..180 is an angle, not a pulse) */
        if (strchr(buf, '.') != NULL) {
            is_ms = true;
        } else if (value > 180.0f) {
            is_us = true;
        } else {
            is_deg = true;
        }
    }

    int pulse;
    if (is_deg) {
        int deg = (int)(value + (value < 0 ? -0.5f : 0.5f));
        if (deg < 0) {
            deg = 0;
        } else if (deg > 180) {
            deg = 180;
        }
        pulse = (angle_to_us != NULL) ? angle_to_us(deg) : deg;
    } else if (is_ms) {
        pulse = (int)(value * 1000.0f + (value < 0 ? -0.5f : 0.5f));
    } else {
        pulse = (int)(value + (value < 0 ? -0.5f : 0.5f));
    }

    if (pulse < min_us) {
        pulse = min_us;
    } else if (pulse > max_us) {
        pulse = max_us;
    }
    return pulse;
}
