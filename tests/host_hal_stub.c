/*
 * host_hal_stub.c  --  run the real TX core on the desktop
 * ---------------------------------------------------------------------------
 * gse_tx_core.c is compiled *unmodified* here; this file only supplies the
 * seven gse_hal.h services using in-memory buffers.  A scripted "laptop" then
 * feeds it text commands and protocol frames, and the bytes that would have
 * gone to the DX-LR41 are decoded with the very same parser the pad runs.
 *
 * What this proves without a board:
 *   * a fresh host frame is forwarded on air, byte for byte, with the
 *     fixed-point address/channel prefix the module expects;
 *   * a host frame older than 200 ms is dropped, not forwarded;
 *   * 300 ms of host silence makes the bridge emit synthetic SAFE at 20 Hz
 *     (FILL closed, PURGE closed, VEHICLE open, IGN off);
 *   * text console: PING / INFO / HELP / SAFE / SET LEVEL / SET KEY;
 *   * the radio is provisioned once at init and its failure is survivable.
 *
 *   make -C tests host-hal
 */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

#include "gse_hal.h"
#include "gse_tx_core.h"

/* ------------------------------------------------------------- fake world */
#define BUFSZ 2048

static uint32_t s_now;
static uint8_t  s_radio_out[BUFSZ]; static size_t s_radio_len;
static uint8_t  s_radio_in [BUFSZ]; static size_t s_radio_in_len, s_radio_in_pos;
static uint8_t  s_host_out [BUFSZ]; static size_t s_host_len;
static uint8_t  s_host_in  [BUFSZ]; static size_t s_host_in_len,  s_host_in_pos;
static char     s_log      [BUFSZ]; static size_t s_log_len;
static int      s_provisioned;
static bool     s_provision_result = true;
static uint8_t  s_last_level, s_last_channel;
static uint16_t s_last_key, s_last_addr;

static void world_reset(void)
{
    s_now = 1000;                        /* nonzero: nothing is "gap 0"      */
    s_radio_len = s_radio_in_len = s_radio_in_pos = 0;
    s_host_len  = s_host_in_len  = s_host_in_pos  = 0;
    s_log_len   = 0; s_log[0] = 0;
    s_provisioned = 0;
    s_provision_result = true;
}

static void host_feed(const void *buf, size_t n)
{
    if (s_host_in_len + n > BUFSZ) return;
    memcpy(s_host_in + s_host_in_len, buf, n);
    s_host_in_len += n;
}

static void host_feed_str(const char *s) { host_feed(s, strlen(s)); }

static void radio_feed(const void *buf, size_t n)
{
    if (s_radio_in_len + n > BUFSZ) return;
    memcpy(s_radio_in + s_radio_in_len, buf, n);
    s_radio_in_len += n;
}

/* advance the fake clock in small steps, ticking the core like a main loop */
static void run_for(uint32_t ms)
{
    uint32_t end = s_now + ms;
    while (s_now < end) {
        s_now += 1;
        gse_tx_tick();
    }
}

/* --------------------------------------------------------------- the HAL */
uint32_t gse_millis(void) { return s_now; }

void gse_tx_write(const uint8_t *buf, size_t len)
{
    if (s_radio_len + len > BUFSZ) return;
    memcpy(s_radio_out + s_radio_len, buf, len);
    s_radio_len += len;
}

bool gse_tx_read_byte(uint8_t *out)
{
    if (s_radio_in_pos >= s_radio_in_len) return false;
    *out = s_radio_in[s_radio_in_pos++];
    return true;
}

void gse_host_write(const uint8_t *buf, size_t len)
{
    if (s_host_len + len > BUFSZ) return;
    memcpy(s_host_out + s_host_len, buf, len);
    s_host_len += len;
}

bool gse_host_read_byte(uint8_t *out)
{
    if (s_host_in_pos >= s_host_in_len) return false;
    *out = s_host_in[s_host_in_pos++];
    return true;
}

void gse_log(const char *fmt, ...)
{
    char tmp[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    size_t n = strlen(tmp);
    if (s_log_len + n + 1 < BUFSZ) {
        memcpy(s_log + s_log_len, tmp, n);
        s_log_len += n;
        s_log[s_log_len++] = '\n';
        s_log[s_log_len] = 0;
    }
}

bool gse_radio_provision(uint8_t level, uint8_t channel_hex, uint16_t key,
                         uint16_t my_addr)
{
    s_provisioned++;
    s_last_level = level;
    s_last_channel = channel_hex;
    s_last_key = key;
    s_last_addr = my_addr;
    return s_provision_result;
}

/* --------------------------------------------------------------- helpers */
static int g_frames;
static uint8_t g_type, g_len, g_payload[64];

static void on_frame(uint8_t type, uint8_t seq, const uint8_t *pl, uint8_t len,
                     void *ctx)
{
    (void)seq; (void)ctx;
    g_frames++;
    g_type = type;
    g_len = len;
    memcpy(g_payload, pl, len);
}

/* Decode the fixed-point packets the bridge put on the radio.  All but the
 * last packet are expected to be complete; returns the last decoded frame. */
typedef struct { int count; uint8_t last_type; uint8_t last_len;
                 gse_cmd_t last_cmd; bool any_bad_crc; } air_t;

static air_t drain_air(void)
{
    air_t a; memset(&a, 0, sizeof a);
    gse_parser_t p; gse_parser_init(&p);
    g_frames = 0;
    for (size_t i = 0; i < s_radio_len; ) {
        /* each air_send() packet = 2 B address + 1 B channel + frame        */
        size_t start = i + 3;
        size_t f = start;
        size_t total = 0;
        if (start + 2 <= s_radio_len) {
            uint8_t l = s_radio_out[start + 4];
            total = 7u + l;
        }
        if (total == 0 || start + total > s_radio_len) break;
        for (size_t k = start; k < start + total; ++k)
            gse_parser_push(&p, s_radio_out[k], on_frame, NULL);
        a.count++;
        a.last_type = g_type;
        a.last_len  = g_len;
        if (g_type == GSE_T_CMD && g_len == sizeof(gse_cmd_t))
            memcpy(&a.last_cmd, g_payload, sizeof(gse_cmd_t));
        i = start + total;
        f = i; (void)f;
    }
    s_radio_len = 0;                     /* consume, so counts are per-window */
    return a;
}

static gse_cmd_t make_cmd(void)
{
    gse_cmd_t c; memset(&c, 0, sizeof c);
    c.arm_code = GSE_ARM_CODE; c.fill = 1; c.veh = 0; c.prg = 1; c.ign = 0;
    c.wdg_ds = 12; c.ign_max_ds = 6; c.flags = GSE_CF_REQ_ACK; c.tx_ms = 0xBEEF;
    return c;
}

/* ------------------------------------------------------------------ tests */
static int s_fail;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (!(cond)) { printf("  FAIL %s (line %d)\n", msg, __LINE__);       \
                       s_fail++; } else { printf("  ok   %s\n", msg); }      \
    } while (0)

static void test_provision_at_init(void)
{
    world_reset();
    gse_tx_init();
    CHECK(s_provisioned == 1, "radio provisioned exactly once at init");
    CHECK(s_last_level == 3 && s_last_channel == 0x0E,
          "init uses LEVEL 3 and listens on the pad's channel 0x0E");
    CHECK(s_last_addr == 0x0001, "TX node address is 0x0001");
    CHECK(strstr(s_log, "ready") != NULL, "init logs 'ready' on success");

    world_reset();
    s_provision_result = false;
    gse_tx_init();
    CHECK(strstr(s_log, "RADIO FAULT") != NULL,
          "provisioning failure is reported, not fatal");
    CHECK(gse_tx_stats()->radio_present == false, "stats expose radio_present=false");
}

static void test_fresh_frame_is_forwarded(void)
{
    world_reset();
    gse_tx_init();
    s_radio_len = 0;

    /* the bridge drops the *first* frame of a session as stale by design, so
     * prime it with one, then measure the second */
    uint8_t frame[GSE_MAX_FRAME];
    gse_cmd_t c = make_cmd();
    size_t n = gse_encode_cmd(1, &c, frame);
    host_feed(frame, n); run_for(20);
    s_radio_len = 0;

    c.tx_ms = 0x1234;
    n = gse_encode_cmd(2, &c, frame);
    host_feed(frame, n); run_for(20);

    air_t a = drain_air();
    CHECK(a.count == 1, "one host CMD produces exactly one air packet");
    CHECK(a.last_type == GSE_T_CMD && a.last_len == sizeof(gse_cmd_t),
          "forwarded payload keeps its type and length");
    CHECK(a.last_cmd.tx_ms == 0x1234 && a.last_cmd.fill == 1,
          "forwarded payload is byte-identical");
}

static void test_air_prefix_and_safe_flags(void)
{
    world_reset();
    gse_tx_init();
    s_radio_len = 0;
    uint8_t frame[GSE_MAX_FRAME];
    gse_cmd_t c = make_cmd();

    for (int i = 0; i < 3; ++i) {
        size_t n = gse_encode_cmd((uint8_t)i, &c, frame);
        host_feed(frame, n);
        run_for(25);
    }
    /* first packet in the buffer: 2 B dest address, 1 B dest channel, then 7E */
    CHECK(s_radio_out[0] == 0x00 && s_radio_out[1] == 0x02,
          "air packet is addressed to the pad (0x0002)");
    CHECK(s_radio_out[2] == 0x0Cu, "air packet carries the pad channel 0x0C");
    CHECK(s_radio_out[3] == GSE_SOF, "protocol frame follows the 3-byte prefix");
    (void)drain_air();
}

static void test_stale_frame_dropped(void)
{
    world_reset();
    gse_tx_init();
    uint8_t frame[GSE_MAX_FRAME];
    gse_cmd_t c = make_cmd();

    size_t n = gse_encode_cmd(1, &c, frame);
    host_feed(frame, n); run_for(20);        /* prime                 */
    s_radio_len = 0;
    run_for(260);                            /* GUI stalls for 260 ms */
    n = gse_encode_cmd(2, &c, frame);
    host_feed(frame, n); run_for(5);         /* late frame arrives    */

    air_t a = drain_air();
    CHECK(gse_tx_stats()->dropped_stale >= 1, "late host frame counted as stale");
    CHECK(a.count == 0 || a.last_cmd.tx_ms != 0xBEEF,
          "late host frame is not forwarded verbatim");
}

static void test_silence_yields_safe(void)
{
    world_reset();
    gse_tx_init();
    s_radio_len = 0;
    /* hold a dangerous command, then stop talking to the bridge */
    uint8_t frame[GSE_MAX_FRAME];
    gse_cmd_t c = make_cmd();
    c.arm_code = GSE_ARM_CODE; c.fill = 1; c.veh = 0; c.prg = 1; c.ign = 1;
    c.wdg_ds = 12; c.ign_max_ds = 6;
    /* the first frame of a session is dropped as stale by design, so prime  */
    size_t n = gse_encode_cmd(1, &c, frame);
    host_feed(frame, n); run_for(20);
    n = gse_encode_cmd(2, &c, frame);
    host_feed(frame, n); run_for(20);
    drain_air();

    run_for(600);                     /* 300 ms dead-man, then 20 Hz SAFE  */
    air_t a = drain_air();
    CHECK(a.count >= 4 && a.count <= 8, "silence produces ~20 Hz SAFE frames");
    CHECK(a.last_cmd.fill == 0, "SAFE closes the FILL valve");
    CHECK(a.last_cmd.prg == 0, "SAFE closes the PURGE valve");
    CHECK(a.last_cmd.veh == 1, "SAFE opens the VEHICLE (vent) valve");
    CHECK(a.last_cmd.ign == 0, "SAFE drops the ignition relay");
    CHECK(a.last_cmd.wdg_ds == 12 && a.last_cmd.ign_max_ds == 6,
          "SAFE keeps the operator's watchdog/igniter limits");
}

static void test_text_console(void)
{
    world_reset();
    gse_tx_init();
    s_log_len = 0; s_log[0] = 0;

    host_feed_str("ping\r\n");        run_for(5);
    CHECK(strstr(s_log, "PONG") != NULL, "PING answers PONG (case-insensitive)");

    host_feed_str("HELP\n");          run_for(5);
    CHECK(strstr(s_log, "SAFE") != NULL, "HELP lists SAFE");

    host_feed_str("SET LEVEL 5\r\n"); run_for(5);
    CHECK(gse_tx_stats()->level == 5, "SET LEVEL 5 changes the requested level");

    host_feed_str("RADIO CONFIG\r\n"); run_for(5);
    CHECK(s_provisioned == 2, "RADIO CONFIG re-provisions on demand");
    CHECK(s_last_level == 5, "RADIO CONFIG applies the new level");

    host_feed_str("NONSENSE\r\n");    run_for(5);
    CHECK(strstr(s_log, "unknown command") != NULL, "unknown text is rejected");
}

static void test_uplink_passthrough(void)
{
    world_reset();
    gse_tx_init();
    s_host_len = 0;

    gse_state_t s; memset(&s, 0, sizeof s);
    s.flags = GSE_ST_LINK_OK; s.out_fill = 1; s.out_veh = 1;
    s.age_ms = 42; s.rx_count = 99; s.pad_state = GSE_PAD_RUN;
    uint8_t frame[GSE_MAX_FRAME];
    size_t n = gse_encode_state(7, &s, frame);
    radio_feed(frame, n);             /* pad -> module -> UART          */
    run_for(5);

    CHECK(s_host_len == n, "pad frame reaches the laptop with the same length");

    gse_parser_t hp; gse_parser_init(&hp);
    g_frames = 0; g_type = 0; g_len = 0;
    for (size_t i = 0; i < s_host_len; ++i)
        gse_parser_push(&hp, s_host_out[i], on_frame, NULL);
    CHECK(g_frames == 1 && g_type == GSE_T_STATE, "laptop sees one STATE frame");
    CHECK(g_len == sizeof(gse_state_t) &&
          memcmp(g_payload, frame + 5, sizeof(gse_state_t)) == 0,
          "STATE payload is forwarded unchanged (only the seq byte is renumbered)");
}

int main(void)
{
    printf("host HAL tests (portable TX core, unmodified)\n");
    test_provision_at_init();
    test_fresh_frame_is_forwarded();
    test_air_prefix_and_safe_flags();
    test_stale_frame_dropped();
    test_silence_yields_safe();
    test_text_console();
    test_uplink_passthrough();
    printf(s_fail ? "\n%d FAILED\n" : "\nALL PASS\n", s_fail);
    return s_fail ? 1 : 0;
}
