/*
 * main.c - dual servo test rig for the NUCLEO-H753ZI
 *
 * Terminal app: type commands to turn servo1 / servo2 / both on or off, aim
 * them at an angle or a pulse width, sweep them, or read status.
 *
 *   on servo1 | on servo2 | on both     start the 50 Hz pulse train (centre)
 *   off servo1 | off servo2 | off both  stop that servo (goes limp)
 *   servo1 90 | servo2 45deg            aim by angle
 *   both 1500us | both 1.5ms            aim by pulse width
 *   center both | sweep both | status | help
 *   shortcuts: 1 2 3 0 9 s ?
 *
 * Console: ST-LINK virtual COM port, 115200 8N1.
 */
#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "console.h"
#include "position.h"
#include "servo.h"

#define LINE_MAX_LEN   64
#define TOK_MAX        8
#define SWEEP_STEP_US  10
#define SWEEP_DELAY_MS 15
#define PROMPT         "servo> "

/* Which servos a command applies to. */
enum target {
    T_NONE = 0,
    T_ONE,              /* t_idx valid  */
    T_BOTH
};

/* ------------------------------------------------------------------------- */
/* Output helpers                                                            */
/* ------------------------------------------------------------------------- */

static void print_banner(void)
{
    console_puts("\n"
                 "==================================================\n"
                 " NUCLEO-H753ZI  -  dual servo test firmware\n"
                 "==================================================\n");
    for (int i = 0; i < SERVO_COUNT; i++) {
        console_printf("  %s : signal on %-4s (CN10-%d)  state: %s\n",
                       servo_name(i), servo_pin_name(i),
                       i == 0 ? 4 : 6,
                       servo_is_on(i) ? "ON" : "off");
    }
    console_printf("  servo profile: %d deg, %d-%d us, centre %d us @ %d Hz\n",
                   SERVO_TRAVEL_DEG, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US,
                   SERVO_DEFAULT_PULSE_US, SERVO_FRAME_HZ);
    console_printf("  console: USART3 PD8/PD9 (ST-LINK VCP) @ %u baud\n",
                   (unsigned)BOARD_CONSOLE_BAUD);
    console_puts("--------------------------------------------------\n"
                 " Type 'help' for the command list.\n"
                 " Quick keys: 1=servo1 on  2=servo2 on  3=both on  0=all off\n"
                 "--------------------------------------------------\n\n");
}

static void print_help(void)
{
    console_printf(
        "\nCommands (case-insensitive)\n"
        "  on SERVO           start pulses at centre (%d us)\n"
        "  off SERVO          stop pulses, signal held low (servo goes limp)\n"
        "  SERVO VALUE        aim at an angle or a pulse width:\n"
        "                       90 / 90deg    -> %d deg travel range\n"
        "                       1.5 / 1.5ms   -> 1500 us\n"
        "                       1500 / 1500us -> 1500 us\n"
        "  center SERVO       back to %d us\n"
        "  sweep SERVO        slow 0 -> %d -> 0 -> %d deg\n"
        "  status             state, pulse width and angle of both servos\n"
        "  help               this text\n"
        "\n"
        "  SERVO is one of: servo1 servo2 both\n"
        "  Shortcuts: 1=servo1 on  2=servo2 on  3=both on  0=all off\n"
        "             9=sweep both  s=status  ?=help\n"
        "\n"
        "Examples:  on both      servo2 45deg     off servo1\n"
        "           both 1.8ms   sweep both       status\n\n",
        SERVO_DEFAULT_PULSE_US, SERVO_TRAVEL_DEG, SERVO_DEFAULT_PULSE_US,
        SERVO_TRAVEL_DEG, SERVO_TRAVEL_DEG);
}

static void print_status(void)
{
    console_puts("\n");
    for (int i = 0; i < SERVO_COUNT; i++) {
        console_printf("  %s  %-4s  state=%-3s  pulse=%4u us  angle=%3d deg\n",
                       servo_name(i), servo_pin_name(i),
                       servo_is_on(i) ? "ON" : "off",
                       (unsigned)servo_pulse_us(i), servo_angle(i));
    }
    console_printf("  uptime %u ms\n\n", (unsigned)board_millis());
}

/* ------------------------------------------------------------------------- */
/* Actions                                                                   */
/* ------------------------------------------------------------------------- */

static void apply_on(enum target t, int idx, int pulse_us)
{
    int first = (t == T_ONE) ? idx : 0;
    int last  = (t == T_ONE) ? idx : SERVO_COUNT - 1;

    for (int i = first; i <= last; i++) {
        servo_on(i, (uint16_t)pulse_us);
        console_printf("  %s -> ON   pulse %u us  (%d deg)\n",
                       servo_name(i), (unsigned)servo_pulse_us(i), servo_angle(i));
    }
}

static void apply_off(enum target t, int idx)
{
    int first = (t == T_ONE) ? idx : 0;
    int last  = (t == T_ONE) ? idx : SERVO_COUNT - 1;

    for (int i = first; i <= last; i++) {
        servo_off(i);
        console_printf("  %s -> OFF  (no pulses, signal low)\n", servo_name(i));
    }
}

static void apply_sweep(enum target t, int idx)
{
    int first = (t == T_ONE) ? idx : 0;
    int last  = (t == T_ONE) ? idx : SERVO_COUNT - 1;

    console_printf("  sweeping %s ... (Ctrl-C is not available mid-sweep)\n",
                   (first == last) ? servo_name(first) : "both servos");

    for (int pulse = SERVO_MIN_PULSE_US; pulse <= SERVO_MAX_PULSE_US; pulse += SWEEP_STEP_US) {
        for (int i = first; i <= last; i++) {
            servo_on(i, (uint16_t)pulse);
        }
        board_delay_ms(SWEEP_DELAY_MS);
    }
    for (int pulse = SERVO_MAX_PULSE_US; pulse >= SERVO_MIN_PULSE_US; pulse -= SWEEP_STEP_US) {
        for (int i = first; i <= last; i++) {
            servo_on(i, (uint16_t)pulse);
        }
        board_delay_ms(SWEEP_DELAY_MS);
    }
    for (int i = first; i <= last; i++) {
        servo_on(i, SERVO_MIN_PULSE_US);
    }
    console_printf("  sweep done (%s at %d us)\n",
                   (first == last) ? servo_name(first) : "both servos",
                   SERVO_MIN_PULSE_US);
}

/* ------------------------------------------------------------------------- */
/* Line handling                                                             */
/* ------------------------------------------------------------------------- */

static char lower(char c)
{
    return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

static bool token_is(const char *tok, const char *word)
{
    while (*tok != '\0' && *word != '\0') {
        if (lower(*tok) != *word) {
            return false;
        }
        tok++;
        word++;
    }
    return *tok == '\0' && *word == '\0';
}

static void lowercase(char *s)
{
    for (; *s != '\0'; s++) {
        *s = lower(*s);
    }
}

static void trim(char **s)
{
    while (**s == ' ' || **s == '\t') {
        (*s)++;
    }
    char *end = *s;
    while (*end != '\0') {
        end++;
    }
    while (end > *s && (end[-1] == ' ' || end[-1] == '\t')) {
        *--end = '\0';
    }
}

/* Split on spaces/tabs in place. */
static int tokenize(char *line, char *tok[], int max)
{
    int n = 0;
    char *p = line;
    while (*p != '\0' && n < max) {
        while (*p == ' ' || *p == '\t') {
            *p++ = '\0';
        }
        if (*p == '\0') {
            break;
        }
        tok[n++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t') {
            p++;
        }
    }
    return n;
}

static void handle_line(char *line)
{
    char *tok[TOK_MAX];
    int   ntok;

    if (line[0] == '\0') {
        return;
    }

    /* Single-key shortcuts. */
    if (line[1] == '\0') {
        switch (line[0]) {
        case '1': apply_on(T_ONE, 0, SERVO_DEFAULT_PULSE_US); return;
        case '2': apply_on(T_ONE, 1, SERVO_DEFAULT_PULSE_US); return;
        case '3': apply_on(T_BOTH, 0, SERVO_DEFAULT_PULSE_US); return;
        case '0': apply_off(T_BOTH, 0); return;
        case '9': apply_sweep(T_BOTH, 0); return;
        case 's': print_status(); return;
        case '?':
        case 'h': print_help(); return;
        default: break;
        }
    }

    ntok = tokenize(line, tok, TOK_MAX);
    if (ntok == 0) {
        return;
    }

    enum target target = T_NONE;
    int  idx = -1;                  /* valid when target == T_ONE */
    int  pulse = -1;
    bool act_on = false, act_off = false, act_center = false;
    bool act_sweep = false, act_status = false, act_help = false;

    /* A token with an explicit unit can only be a position, never a servo
     * name - find it first so "servo1 1500us" is unambiguous. */
    const char *unit_tok = 0;
    for (int i = 0; i < ntok; i++) {
        if (position_has_unit_suffix(tok[i])) {
            unit_tok = tok[i];
            break;
        }
    }

    for (int i = 0; i < ntok; i++) {
        const char *t = tok[i];

        if (unit_tok != 0 && t == unit_tok) {
            continue;
        }
        if (token_is(t, "servo1") || token_is(t, "servo_1") || token_is(t, "s1") ||
            token_is(t, "1")) {
            target = T_ONE;
            idx = 0;
        } else if (token_is(t, "servo2") || token_is(t, "servo_2") || token_is(t, "s2") ||
                   token_is(t, "2")) {
            target = T_ONE;
            idx = 1;
        } else if (token_is(t, "both") || token_is(t, "all") || token_is(t, "servos") ||
                   token_is(t, "servo")) {
            target = T_BOTH;
        } else if (token_is(t, "on") || token_is(t, "start") || token_is(t, "enable")) {
            act_on = true;
        } else if (token_is(t, "off") || token_is(t, "stop") || token_is(t, "disable")) {
            act_off = true;
        } else if (token_is(t, "center") || token_is(t, "centre") || token_is(t, "mid")) {
            act_center = true;
        } else if (token_is(t, "sweep") || token_is(t, "test")) {
            act_sweep = true;
        } else if (token_is(t, "status") || token_is(t, "state") || token_is(t, "list")) {
            act_status = true;
        } else if (token_is(t, "help") || token_is(t, "menu")) {
            act_help = true;
        } else {
            int parsed = position_parse_pulse_us(t, SERVO_MIN_PULSE_US,
                                                SERVO_MAX_PULSE_US,
                                                servo_angle_to_pulse_us_int);
            if (parsed > 0) {
                pulse = parsed;
            } else {
                console_printf("  unknown token '%s' (try 'help')\n", t);
                return;
            }
        }
    }

    if (unit_tok != 0) {
        int parsed = position_parse_pulse_us(unit_tok, SERVO_MIN_PULSE_US,
                                             SERVO_MAX_PULSE_US,
                                             servo_angle_to_pulse_us_int);
        if (parsed <= 0) {
            console_printf("  unknown token '%s' (try 'help')\n", unit_tok);
            return;
        }
        pulse = parsed;
    }

    if (act_help) {
        print_help();
        return;
    }
    if (act_status) {
        print_status();
        return;
    }

    if (target == T_NONE) {
        if (act_off && !act_on) {
            target = T_BOTH;        /* bare "off" = everything off */
        } else if (act_sweep || act_center || act_on || pulse > 0) {
            target = T_BOTH;        /* bare "on" / "90" = both servos */
        } else {
            console_puts("  nothing to do (try 'help')\n");
            return;
        }
    }

    if (act_off) {
        apply_off(target, idx);
    } else if (act_sweep) {
        apply_sweep(target, idx);
    } else if (act_center) {
        apply_on(target, idx, SERVO_DEFAULT_PULSE_US);
    } else {
        apply_on(target, idx, (pulse > 0) ? pulse : SERVO_DEFAULT_PULSE_US);
    }
}

/* ------------------------------------------------------------------------- */
/* main                                                                      */
/* ------------------------------------------------------------------------- */

int main(void)
{
    static char line[LINE_MAX_LEN];

    board_dwt_init();
    board_clock_init();
    console_init();
    servo_init();

    print_banner();

    for (;;) {
        console_puts(PROMPT);

        int len = console_read_line(line, (int)sizeof(line));
        if (len < 0) {
            console_puts("  (line too long or cancelled)\n");
            continue;
        }

        char *cmd = line;
        trim(&cmd);
        lowercase(cmd);
        handle_line(cmd);
    }
}
