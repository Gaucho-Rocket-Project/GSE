/*
 * ESP32-S3 dual SG90 servo test rig
 * ---------------------------------
 * Serial-console firmware: type commands to turn servo1 / servo2 / both on or
 * off, aim them, sweep them, or read status.
 *
 * Console: USB Serial/JTAG by default (the USB port on the dev board).
 *          Build with `make build-uart` for the UART0 (COM) port instead.
 *
 * Commands (case-insensitive):
 *   help | ?                     show the menu
 *   status | s                   show both servos
 *   on servo1 | servo2 | both    start pulses at centre (1.5 ms)
 *   off servo1 | servo2 | both   stop pulses for that servo
 *   servo1 90 | servo2 1.2ms     aim by angle (deg) or pulse width (ms/us/deg)
 *   both 45deg | both 1.8ms      aim both
 *   center both                  re-centre at 1.5 ms
 *   sweep both                   slow sweep 0 -> 180 -> 0
 *
 * Numeric shortcut menu: 1 = servo1 on, 2 = servo2 on, 3 = both on,
 *                        0 = both off, 9 = sweep both, s = status, ? = help
 */

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "console_io.h"
#include "position.h"
#include "servo.h"

static const char *TAG = "app";

#define LINE_MAX_LEN 64
#define TOK_MAX      8
#define SWEEP_STEP_US   10
#define SWEEP_DELAY_MS  15

/* Which servos a command applies to */
enum target {
    T_NONE = 0,
    T_ONE,          /* t_idx valid */
    T_BOTH,
};

static void print_banner(void)
{
    printf("\n"
           "==================================================\n"
           " ESP32-S3  -  dual SG90 servo test firmware\n"
           "==================================================\n");
    for (int i = 0; i < SERVO_COUNT; i++) {
        printf("  %s : signal on GPIO%-2d  (state: %s)\n",
               servo_name(i), servo_gpio(i), servo_is_on(i) ? "ON" : "off");
    }
    printf("  servo type: %s, %d deg, %d-%d us\n",
           SERVO_TYPE_NAME, SERVO_TRAVEL_DEG, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    printf("--------------------------------------------------\n"
           " Type 'help' for commands.\n"
           "   on servo1 | on servo2 | on both\n"
           "   off servo1 | off servo2 | off both\n"
           "   servo1 90   (angle)  |  servo1 1.8ms  (pulse)\n"
           "   sweep both  |  status  |  help\n"
           " Shortcuts: 1=servo1 on  2=servo2 on  3=both on  0=all off\n"
           "--------------------------------------------------\n\n");
}

static void print_help(void)
{
    printf(
        "\nCommands\n"
        "  on SERVO          pulses on at centre (1.5 ms). SERVO = servo1 | servo2 | both\n"
        "  off SERVO         pulses off, signal parked low (servo goes limp)\n"
        "  SERVO VALUE       aim; VALUE is an angle or a pulse width:\n"
        "                      90 / 90deg      -> 90 degrees  (0-%d deg travel)\n"
        "                      1.5 / 1.5ms     -> 1500 us\n"
        "                      1500 / 1500us   -> 1500 us\n"
        "  center SERVO      back to 1.5 ms\n"
        "  sweep SERVO       slow 0 -> %d -> 0 sweep\n"
        "  status            show both servos\n"
        "  help              this text\n"
        "Shortcuts: 1 2 3 0 9 s ?   (enter one character per line)\n"
        "Examples:  'on both'   'servo2 45deg'   'off servo1'   'both 1.8ms'\n\n",
        SERVO_TRAVEL_DEG, SERVO_TRAVEL_DEG);
}

static void print_status(void)
{
    printf("\n");
    for (int i = 0; i < SERVO_COUNT; i++) {
        int      angle = (int)(((int)servo_pulse_us(i) - SERVO_MIN_PULSE_US) * SERVO_TRAVEL_DEG /
                               (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US));
        printf("  %s  GPIO%-2d  state=%-3s  pulse=%4u us  (~%3d deg)\n",
               servo_name(i), servo_gpio(i),
               servo_is_on(i) ? "ON" : "off",
               (unsigned)servo_pulse_us(i), angle);
    }
    printf("\n");
}

static void lower_in_place(char *s)
{
    for (; *s; s++) {
        *s = (char)tolower((unsigned char)*s);
    }
}

/* Parse "on"/"off"/... style words */
static bool token_is(const char *tok, const char *word)
{
    return strcmp(tok, word) == 0;
}

static void apply_on(enum target t, int idx, int pulse_us)
{
    int first = (t == T_ONE) ? idx : 0;
    int last  = (t == T_ONE) ? idx : SERVO_COUNT - 1;

    for (int i = first; i <= last; i++) {
        if (servo_on(i, (uint16_t)pulse_us) == ESP_OK) {
            printf("  %s -> ON, pulse %d us\n", servo_name(i), pulse_us);
        } else {
            printf("  %s -> failed to start!\n", servo_name(i));
        }
    }
}

static void apply_off(enum target t, int idx)
{
    int first = (t == T_ONE) ? idx : 0;
    int last  = (t == T_ONE) ? idx : SERVO_COUNT - 1;

    for (int i = first; i <= last; i++) {
        servo_off(i);
        printf("  %s -> OFF (no pulses)\n", servo_name(i));
    }
}

static void apply_sweep(enum target t, int idx)
{
    int first = (t == T_ONE) ? idx : 0;
    int last  = (t == T_ONE) ? idx : SERVO_COUNT - 1;

    printf("  sweeping %s ...\n", first == last ? servo_name(first) : "both servos");

    for (int pulse = SERVO_MIN_PULSE_US; pulse <= SERVO_MAX_PULSE_US; pulse += SWEEP_STEP_US) {
        for (int i = first; i <= last; i++) {
            servo_on(i, (uint16_t)pulse);
        }
        vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY_MS));
    }
    for (int pulse = SERVO_MAX_PULSE_US; pulse >= SERVO_MIN_PULSE_US; pulse -= SWEEP_STEP_US) {
        for (int i = first; i <= last; i++) {
            servo_on(i, (uint16_t)pulse);
        }
        vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY_MS));
    }
    printf("  sweep done (%s back at %d us)\n",
           first == last ? servo_name(first) : "both",
           SERVO_MIN_PULSE_US);
}

/* Handle one line of input. */
static void handle_line(char *line)
{
    char *tok[TOK_MAX];
    int   ntok = 0;

    /* single-character menu shortcuts */
    if (strlen(line) == 1) {
        switch (line[0]) {
            case '1': apply_on(T_ONE, 0, SERVO_DEFAULT_PULSE_US); return;
            case '2': apply_on(T_ONE, 1, SERVO_DEFAULT_PULSE_US); return;
            case '3': apply_on(T_BOTH, 0, SERVO_DEFAULT_PULSE_US); return;
            case '0': apply_off(T_BOTH, 0); return;
            case '9': apply_sweep(T_BOTH, 0); return;
            case 's': print_status(); return;
            case '?': case 'h': print_help(); return;
            default: break;
        }
    }

    /* tokenise */
    char *save = NULL;
    for (char *p = strtok_r(line, " \t", &save); p != NULL && ntok < TOK_MAX;
         p = strtok_r(NULL, " \t", &save)) {
        tok[ntok++] = p;
    }
    if (ntok == 0) {
        return;
    }

    enum target target = T_NONE;
    int         idx    = -1;      /* valid when target == T_ONE */
    int         pulse  = -1;
    bool        action_on = false, action_off = false, action_center = false;
    bool        action_sweep = false, action_status = false, action_help = false;

    /*
     * Two passes over the tokens:
     *   1. find a position written with an explicit unit ("1500us", "1.5ms",
     *      "90deg") - those can never be a servo selector;
     *   2. interpret the remaining tokens as keywords / servo names / bare
     *      numbers.
     * That way "servo1 1500" is unambiguous and "1500us" is a position even
     * though 1500 could look like a pulse width.
     */
    const char *explicit_unit_tok = NULL;
    for (int i = 0; i < ntok; i++) {
        if (position_has_unit_suffix(tok[i])) {
            explicit_unit_tok = tok[i];
            break;
        }
    }

    for (int i = 0; i < ntok; i++) {
        const char *t = tok[i];
        if (position_has_unit_suffix(t)) {
            continue;       /* handled by explicit_unit_tok above (first one wins) */
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
        } else if (token_is(t, "on") || token_is(t, "start")) {
            action_on = true;
        } else if (token_is(t, "off") || token_is(t, "stop")) {
            action_off = true;
        } else if (token_is(t, "center") || token_is(t, "centre") || token_is(t, "mid")) {
            action_center = true;
        } else if (token_is(t, "sweep") || token_is(t, "test")) {
            action_sweep = true;
        } else if (token_is(t, "status") || token_is(t, "state") || token_is(t, "list")) {
            action_status = true;
        } else if (token_is(t, "help") || token_is(t, "h") || token_is(t, "?")) {
            action_help = true;
        } else {
            int parsed = position_parse_pulse_us(t, SERVO_MIN_PULSE_US,
                                                 SERVO_MAX_PULSE_US,
                                                 servo_angle_to_pulse_us);
            if (parsed > 0) {
                pulse = parsed;
            } else {
                printf("  unknown token '%s' (try 'help')\n", t);
                return;
            }
        }
    }

    if (explicit_unit_tok != NULL) {
        int parsed = position_parse_pulse_us(explicit_unit_tok, SERVO_MIN_PULSE_US,
                                             SERVO_MAX_PULSE_US, servo_angle_to_pulse_us);
        if (parsed <= 0) {
            printf("  unknown token '%s' (try 'help')\n", explicit_unit_tok);
            return;
        }
        pulse = parsed;
    }

    if (action_help) {
        print_help();
        return;
    }
    if (action_status) {
        print_status();
        return;
    }
    if (target == T_NONE) {
        if (action_off && !action_on) {
            target = T_BOTH;           /* bare "off" turns everything off */
        } else if (action_sweep) {
            target = T_BOTH;
        } else if (action_center) {
            target = T_BOTH;
        } else if (action_on || pulse > 0) {
            target = T_BOTH;           /* "on" / "90" with no target = both */
        } else {
            printf("  nothing to do (try 'help')\n");
            return;
        }
    }

    if (action_off) {
        apply_off(target, idx);
    } else if (action_sweep) {
        apply_sweep(target, idx);
    } else if (action_center) {
        apply_on(target, idx, SERVO_DEFAULT_PULSE_US);
    } else {
        /* plain "on", or a position value, or both */
        int p = (pulse > 0) ? pulse : SERVO_DEFAULT_PULSE_US;
        apply_on(target, idx, p);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "booting ESP32-S3 dual-SG90 servo test firmware");

    esp_err_t err = servo_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "servo_init failed: %s", esp_err_to_name(err));
    }

    console_io_init();
    print_banner();

    static char line[LINE_MAX_LEN];

    for (;;) {
        console_io_write_str("servo> ");
        int len = console_io_read_line(line, sizeof(line));
        if (len < 0) {
            continue;
        }
        lower_in_place(line);

        /* trim leading/trailing spaces */
        char *cmd = line;
        while (*cmd == ' ') {
            cmd++;
        }
        size_t l = strlen(cmd);
        while (l > 0 && cmd[l - 1] == ' ') {
            cmd[--l] = '\0';
        }
        if (*cmd == '\0') {
            continue;
        }

        handle_line(cmd);
        printf("\n");       /* keep the next prompt on its own line */
    }
}
