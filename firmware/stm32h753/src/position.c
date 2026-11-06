/*
 * position.c - turn a typed token such as "90", "45deg", "1.5ms" or "1500us"
 * into a servo pulse width in microseconds.
 *
 * Pure C, no libc, no hardware: see tests/test_position_stm32.c for the
 * host-side test suite that exercises this exact file.
 */
#include <stddef.h>

#include "position.h"

#define POSITION_MAX_DEG   270      /* widest DS3225 travel variant        */
#define POSITION_MAX_INPUT 32       /* reject absurdly long tokens         */

static bool is_digit(char c)
{
    return c >= '0' && c <= '9';
}

static bool is_space(char c)
{
    return c == ' ' || c == '\t';
}

static bool str_eq(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0') {
        if (*a != *b) {
            return false;
        }
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

/*
 * Parse "<sign><digits>[.<digits>]" starting at @p s.
 * Returns the position just past the number, or NULL if there is no number.
 * The value is split into @p whole and @p frac (up to 4 decimals) so that no
 * floating point or strtod is needed anywhere in the firmware.
 */
static const char *scan_number(const char *s, int *whole, int *frac, int *frac_digits)
{
    *whole = 0;
    *frac = 0;
    *frac_digits = 0;

    if (*s == '+' || *s == '-') {
        s++;
    }
    if (!is_digit(*s)) {
        return NULL;
    }
    while (is_digit(*s)) {
        if (*whole < 1000000) {          /* saturate, never overflow */
            *whole = *whole * 10 + (*s - '0');
        }
        s++;
    }
    if (*s == '.') {
        s++;
        if (!is_digit(*s)) {
            return NULL;                 /* "1." / "1.ms" is not a number */
        }
        while (is_digit(*s)) {
            if (*frac_digits < 4) {
                *frac = *frac * 10 + (*s - '0');
                (*frac_digits)++;
            }
            s++;
        }
    }
    return s;
}

static bool suffix_is_deg(const char *p)
{
    return str_eq(p, "deg") || str_eq(p, "d") || str_eq(p, "degree") ||
           str_eq(p, "degrees") || str_eq(p, "\xc2\xb0");   /* UTF-8 "°" */
}

static bool suffix_is_ms(const char *p)
{
    return str_eq(p, "ms") || str_eq(p, "msec");
}

static bool suffix_is_us(const char *p)
{
    return str_eq(p, "us") || str_eq(p, "usec") || str_eq(p, "\xc2\xb5s");
}

bool position_has_unit_suffix(const char *tok)
{
    int w, f, fd;

    if (tok == NULL || *tok == '\0') {
        return false;
    }
    while (is_space(*tok)) {
        tok++;
    }

    const char *p = scan_number(tok, &w, &f, &fd);
    if (p == NULL) {
        return false;                    /* "servo1" is not a position */
    }
    while (is_space(*p)) {
        p++;
    }
    if (*p == '\0') {
        return false;                    /* plain number, no unit */
    }
    return suffix_is_deg(p) || suffix_is_ms(p) || suffix_is_us(p);
}

int position_parse_pulse_us(const char *tok, int min_us, int max_us,
                            int (*angle_to_us)(int))
{
    int whole, frac, frac_digits;
    bool is_deg = false, is_ms = false;

    if (tok == NULL || *tok == '\0') {
        return -1;
    }

    /* Copy into a trimmed local buffer so " 1500 us " behaves like "1500us"
     * and a trailing '\n' can never confuse the suffix check. */
    char buf[POSITION_MAX_INPUT];
    size_t len = 0;
    while (tok[len] != '\0') {
        if (len + 1 >= sizeof(buf)) {
            return -1;                   /* absurdly long token */
        }
        buf[len] = tok[len];
        len++;
    }
    buf[len] = '\0';
    while (len > 0 && is_space(buf[len - 1])) {
        buf[--len] = '\0';
    }

    const char *num = buf;
    while (is_space(*num)) {
        num++;
    }
    bool negative = (*num == '-');

    const char *p = scan_number(num, &whole, &frac, &frac_digits);
    if (p == NULL) {
        return -1;
    }
    while (is_space(*p)) {
        p++;
    }

    if (*p == '\0') {
        /* No unit given: infer it from the magnitude.
         *   "1.5"  -> milliseconds (it has a decimal point)
         *   "1500" -> microseconds (bigger than any angle could be)
         *   "90"   -> degrees      (0..270 is an angle) */
        if (frac_digits > 0) {
            is_ms = true;
        } else if (whole > POSITION_MAX_DEG) {
            /* microseconds: handled by the else branch below */
        } else {
            is_deg = true;
        }
    } else if (suffix_is_deg(p)) {
        is_deg = true;
    } else if (suffix_is_ms(p)) {
        is_ms = true;
    } else if (suffix_is_us(p)) {
        /* microseconds: nothing to scale */
    } else {
        return -1;                       /* trailing garbage: "90xyz" */
    }

    if (negative) {
        whole = 0;                       /* clamp "-5" to the low end */
        frac = 0;
    }

    int pulse;
    if (is_deg) {
        int deg = whole;
        if (frac_digits > 0 && frac >= 5) {
            deg++;                       /* round .5 deg up */
        }
        pulse = (angle_to_us != NULL) ? angle_to_us(deg) : deg;
    } else if (is_ms) {
        static const int pow10[5] = { 1, 10, 100, 1000, 10000 };
        pulse = whole * 1000;
        if (frac_digits > 0) {
            pulse += frac * pow10[4 - frac_digits] / 10;   /* + us */
        }
    } else {
        pulse = whole;
    }

    if (pulse < min_us) {
        pulse = min_us;
    } else if (pulse > max_us) {
        pulse = max_us;
    }
    return pulse;
}
