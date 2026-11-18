/*
 * gse_tx_core.c  --  platform-independent TX-node / laptop-bridge logic
 * ---------------------------------------------------------------------------
 * This is the behaviour that used to live inside tx_node.cpp, rewritten as
 * plain C with no Arduino dependencies so that the *same* file drives
 *
 *      ESP32-S3  (firmware/src/tx/tx_platform_esp32.cpp)
 *      STM32H753 (firmware/stm32/tx_platform_stm32.c)
 *
 * Behaviour (unchanged from the ESP32 implementation, verified by the same
 * protocol tests):
 *
 *   * one serial port carries both directions; binary frames start with 0x7E,
 *     text lines are \n-terminated;
 *   * the host must send frames at least every 200 ms -- the first frame after
 *     a longer gap is treated as stale and dropped;
 *   * if the host is silent for 300 ms the bridge synthesises SAFE commands at
 *     20 Hz (FILL closed, PURGE closed, VEHICLE open, IGN off) so a crashed
 *     GUI or a yanked cable cannot hold a valve open;
 *   * text console: PING, INFO, HELP, SAFE, SET LEVEL n, SET KEY n, RADIO CONFIG
 *
 * The pad node remains the only place where safety decisions are made; this
 * code only paces and filters traffic.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gse_protocol.h"
#include "gse_hal.h"
#include "gse_tx_core.h"

/* The portable core cannot include config.h (it pulls in platform headers), so
 * the three build-specific values it needs are supplied by the platform build.
 * A platform may override them with -D on the command line. */
#ifndef GSE_FW_MAJOR
#define GSE_FW_MAJOR   1u
#endif
#ifndef GSE_FW_MINOR
#define GSE_FW_MINOR   0u
#endif
#ifndef GSE_PLATFORM_NAME
#define GSE_PLATFORM_NAME "esp32"
#endif
#ifndef GSE_TX_LEVEL_DEFAULT
#define GSE_TX_LEVEL_DEFAULT  3u          /* AT+LEVEL 3 = 10 kbit/s LoRa     */
#endif
#ifndef GSE_TX_KEY_DEFAULT
#define GSE_TX_KEY_DEFAULT    51042u
#endif

#define LINK_TIMEOUT_MS   300u
#define CMD_MAX_AGE_MS    200u
#define CMD_PERIOD_MS     50u
#define TX_CH_SELF        0x0Eu   /* listens here; must equal GSE_CH_SELF(1) */
#define TX_CH_PEER        0x0Cu   /* transmits here (pad listens)            */
#define TX_ADDR           0x0001u
#define PAD_ADDR          0x0002u

static gse_parser_t s_air, s_host;
static gse_cmd_t    s_last_cmd;
static uint32_t     s_last_host_ms;
static uint32_t     s_last_safe_ms;
static uint8_t      s_seq_air, s_seq_host;
static uint32_t     s_dropped_stale, s_forwarded, s_rx_frames;
static uint8_t      s_level = GSE_TX_LEVEL_DEFAULT;
static uint16_t     s_key = GSE_TX_KEY_DEFAULT;
static char         s_line[96];
static uint8_t      s_line_len;
static bool         s_in_binary;
static bool         s_radio_ok;
static gse_tx_stats_t s_stats;

/* ------------------------------------------------------------------ uplink */
static void air_send(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t frame[GSE_MAX_FRAME];
    uint8_t pkt[GSE_MAX_FRAME + 4];
    size_t n = gse_encode(type, s_seq_air++, payload, len, frame);
    /* fixed-point mode: destination address (2 B) + destination channel (1 B) */
    pkt[0] = (uint8_t)(PAD_ADDR >> 8);
    pkt[1] = (uint8_t)(PAD_ADDR & 0xFF);
    pkt[2] = TX_CH_PEER;
    memcpy(&pkt[3], frame, n);
    gse_tx_write(pkt, n + 3);
    s_stats.sent_frames++;
}

static void air_safe_command(void)
{
    gse_cmd_t c;
    memset(&c, 0, sizeof c);
    c.veh        = 1;                                   /* VEHICLE OPEN   */
    c.wdg_ds     = s_last_cmd.wdg_ds ? s_last_cmd.wdg_ds : GSE_DEF_WDG_DS;
    c.ign_max_ds = s_last_cmd.ign_max_ds ? s_last_cmd.ign_max_ds
                                         : GSE_DEF_IGN_MAX_DS;
    air_send(GSE_T_CMD, &c, sizeof c);
    s_stats.safe_frames++;
}

/* ---------------------------------------------------------------- downlink */
static void host_send(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t frame[GSE_MAX_FRAME];
    size_t n = gse_encode(type, s_seq_host++, payload, len, frame);
    gse_host_write(frame, n);
}

static void on_air(uint8_t type, uint8_t seq, const uint8_t *pl, uint8_t len, void *ctx)
{
    (void)seq; (void)ctx;
    s_rx_frames++;
    host_send(type, pl, len);                 /* pad -> laptop, verbatim   */
    if (type == GSE_T_STATE) s_stats.rx_state++;
    if (type == GSE_T_EVENT) s_stats.rx_event++;
}

static void on_host(uint8_t type, uint8_t seq, const uint8_t *pl, uint8_t len, void *ctx)
{
    (void)seq; (void)ctx;
    uint32_t now = gse_millis();
    uint32_t gap = s_last_host_ms ? (now - s_last_host_ms) : 0xFFFFFFFFu;
    s_last_host_ms = now;

    switch (type) {
    case GSE_T_CMD:
        if (len != sizeof(gse_cmd_t)) { s_stats.bad_host_frames++; return; }
        if (gap > CMD_MAX_AGE_MS) { s_dropped_stale++; return; }
        memcpy(&s_last_cmd, pl, sizeof s_last_cmd);
        s_forwarded++;
        air_send(GSE_T_CMD, pl, len);
        break;
    case GSE_T_CFG:
        if (len == sizeof(gse_cfgs_t)) air_send(GSE_T_CFG, pl, len);
        break;
    case GSE_T_HELLO:
        air_send(GSE_T_HELLO, pl, len);
        break;
    default:
        s_stats.bad_host_frames++;
        break;
    }
}

/* ------------------------------------------------------------ text console */
static void upcase(char *s)
{
    for (; *s; ++s) if (*s >= 'a' && *s <= 'z') *s = (char)(*s - 32);
}

static void handle_line(char *line)
{
    upcase(line);
    if (!strcmp(line, "PING")) {
        gse_log("PONG %lu", (unsigned long)gse_millis());
    } else if (!strcmp(line, "INFO")) {
        gse_log("TX %s fw %u.%u level=%u key=%u fwd=%lu stale=%lu uptime=%lus",
                GSE_PLATFORM_NAME, GSE_FW_MAJOR, GSE_FW_MINOR, s_level, s_key,
                (unsigned long)s_forwarded, (unsigned long)s_dropped_stale,
                (unsigned long)(gse_millis() / 1000u));
    } else if (!strcmp(line, "SAFE")) {
        s_last_host_ms = 0;                    /* force bridge-safe mode    */
        gse_log("SAFE forced");
    } else if (!strncmp(line, "SET LEVEL ", 10)) {
        s_level = (uint8_t)(line[10] - '0');
        if (s_level > 8) s_level = 8;
        gse_log("level=%u (send RADIO CONFIG to apply)", s_level);
    } else if (!strncmp(line, "SET KEY ", 8)) {
        s_key = (uint16_t)atoi(line + 8);
        gse_log("key=%u (send RADIO CONFIG to apply)", s_key);
    } else if (!strcmp(line, "RADIO CONFIG")) {
        s_radio_ok = gse_radio_provision(s_level, TX_CH_SELF, s_key, TX_ADDR);
        if (s_radio_ok) gse_log("RADIO OK level=%u", s_level);
        else            gse_log("RADIO FAULT");
    } else if (!strcmp(line, "HELP")) {
        gse_log("cmds: PING INFO SAFE SET LEVEL n SET KEY n RADIO CONFIG HELP");
    } else if (line[0]) {
        gse_log("ERR unknown command");
    }
}

/* -------------------------------------------------------------------- API */
void gse_tx_init(void)
{
    memset(&s_last_cmd, 0, sizeof s_last_cmd);
    s_last_cmd.veh        = 1;
    s_last_cmd.wdg_ds     = GSE_DEF_WDG_DS;
    s_last_cmd.ign_max_ds = GSE_DEF_IGN_MAX_DS;
    gse_parser_init(&s_air);
    gse_parser_init(&s_host);

    s_radio_ok = gse_radio_provision(s_level, TX_CH_SELF, s_key, TX_ADDR);
    gse_log("TX %s fw %u.%u %s", GSE_PLATFORM_NAME, GSE_FW_MAJOR, GSE_FW_MINOR,
            s_radio_ok ? "ready" : "RADIO FAULT");
}

void gse_tx_tick(void)
{
    uint32_t now = gse_millis();
    uint8_t b;

    /* host serial: share the port between binary frames and text lines */
    while (gse_host_read_byte(&b)) {
        if (b == GSE_SOF) { s_in_binary = true; s_line_len = 0; }
        if (s_in_binary) {
            gse_parser_push(&s_host, b, on_host, NULL);
            if (s_host.len == 0 && b != GSE_SOF) s_in_binary = false;
            continue;
        }
        if (b == '\n' || b == '\r') {
            if (s_line_len) { s_line[s_line_len] = 0; handle_line(s_line); s_line_len = 0; }
            continue;
        }
        if (b >= 0x20 && b < 0x7F) {
            if (s_line_len < sizeof s_line - 1) s_line[s_line_len++] = (char)b;
        } else {
            s_line_len = 0;
        }
    }

    /* radio */
    while (gse_tx_read_byte(&b)) gse_parser_push(&s_air, b, on_air, NULL);

    /* laptop watchdog -> synthetic safe stream at 20 Hz */
    bool alive = s_last_host_ms && (now - s_last_host_ms) < LINK_TIMEOUT_MS;
    if (!alive && (now - s_last_safe_ms) >= CMD_PERIOD_MS) {
        s_last_safe_ms = now;
        air_safe_command();
    }
}

const gse_tx_stats_t *gse_tx_stats(void)
{
    s_stats.dropped_stale   = s_dropped_stale;
    s_stats.forwarded_cmd   = s_forwarded;
    s_stats.rx_frames       = s_rx_frames;
    s_stats.radio_present   = s_radio_ok;
    s_stats.level           = s_level;
    return &s_stats;
}
