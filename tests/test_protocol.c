/* Host-side unit tests for gse_protocol.h  --  build: make -C tests */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "../firmware/src/common/gse_protocol.h"

static int g_frames = 0;
static uint8_t g_last_type = 0;
static uint8_t g_last_payload[64];
static uint8_t g_last_len = 0;

static void on_frame(uint8_t type, uint8_t seq, const uint8_t *pl, uint8_t len, void *ctx)
{
    (void)seq; (void)ctx;
    g_frames++;
    g_last_type = type;
    g_last_len = len;
    memcpy(g_last_payload, pl, len);
}

static void test_crc_known_vector(void)
{
    /* CRC-16/CCITT-FALSE("123456789") = 0x29B1 */
    const uint8_t v[] = "123456789";
    assert(gse_crc16(v, 9) == 0x29B1);
    printf("  ok  crc vector\n");
}

static void test_roundtrip_cmd(void)
{
    gse_cmd_t c;
    memset(&c, 0, sizeof c);
    c.arm_code = GSE_ARM_CODE; c.fill = 1; c.veh = 1; c.prg = 0; c.ign = 1;
    c.wdg_ds = 12; c.ign_max_ds = 6; c.flags = GSE_CF_REQ_ACK; c.tx_ms = 0xBEEF;

    uint8_t buf[GSE_MAX_FRAME];
    size_t n = gse_encode_cmd(9, &c, buf);
    assert(n == 7 + sizeof(gse_cmd_t));
    assert(buf[0] == GSE_SOF && buf[1] == GSE_PROTO_VERSION && buf[2] == GSE_T_CMD);

    gse_parser_t p; gse_parser_init(&p); g_frames = 0;
    for (size_t i = 0; i < n; ++i) gse_parser_push(&p, buf[i], on_frame, NULL);
    assert(g_frames == 1 && g_last_type == GSE_T_CMD && g_last_len == sizeof(gse_cmd_t));

    gse_cmd_t d; memcpy(&d, g_last_payload, sizeof d);
    assert(d.arm_code == GSE_ARM_CODE && d.fill == 1 && d.ign == 1 && d.tx_ms == 0xBEEF);
    printf("  ok  cmd roundtrip (%zu bytes on air)\n", n);
}

static void test_roundtrip_state(void)
{
    gse_state_t s; memset(&s, 0, sizeof s);
    s.flags = GSE_ST_LINK_OK | GSE_ST_HELLO_OK;
    s.out_fill = 1; s.out_veh = 1; s.out_ign = 0; s.age_ms = 123;
    s.rx_count = 4096; s.crc_err = 3; s.rssi_dbm = -84;
    s.pad_state = GSE_PAD_RUN; s.fw_major = 1; s.fw_minor = 0;

    uint8_t buf[GSE_MAX_FRAME];
    size_t n = gse_encode_state(1, &s, buf);
    gse_parser_t p; gse_parser_init(&p); g_frames = 0;
    for (size_t i = 0; i < n; ++i) gse_parser_push(&p, buf[i], on_frame, NULL);
    assert(g_frames == 1 && g_last_len == sizeof(gse_state_t));
    gse_state_t d; memcpy(&d, g_last_payload, sizeof d);
    assert(d.rssi_dbm == -84 && d.age_ms == 123 && d.rx_count == 4096);
    assert(sizeof(gse_state_t) == 24 && sizeof(gse_cmd_t) == 12 && sizeof(gse_cfgs_t) == 8);
    printf("  ok  state roundtrip (%zu bytes on air)\n", n);
}

static void test_corruption_recovery(void)
{
    gse_cmd_t c; memset(&c, 0, sizeof c); c.fill = 1; c.tx_ms = 7;
    uint8_t buf[GSE_MAX_FRAME];
    size_t n = gse_encode_cmd(1, &c, buf);

    gse_parser_t p; gse_parser_init(&p); g_frames = 0;
    /* leading garbage that contains SOF-like bytes */
    const uint8_t junk[] = {0x00, 0x7E, 0x7E, 0xFF, 0x12};
    for (size_t i = 0; i < sizeof junk; ++i) gse_parser_push(&p, junk[i], on_frame, NULL);
    /* a corrupted copy of the frame */
    for (size_t i = 0; i < n; ++i) {
        uint8_t b = buf[i];
        if (i == 6) b ^= 0x40;              /* flip a payload bit */
        gse_parser_push(&p, b, on_frame, NULL);
    }
    assert(g_frames == 0);
    /* then two good frames back-to-back */
    for (size_t i = 0; i < n; ++i) gse_parser_push(&p, buf[i], on_frame, NULL);
    for (size_t i = 0; i < n; ++i) gse_parser_push(&p, buf[i], on_frame, NULL);
    assert(g_frames == 2);
    assert(p.frames_bad >= 1);
    printf("  ok  corruption + resync (bad=%lu, ok=%lu)\n",
           (unsigned long)p.frames_bad, (unsigned long)p.frames_ok);
}

static void test_failsafe_matrix(void)
{
    gse_cfg_t cfg = { GSE_DEF_WDG_DS, GSE_DEF_IGN_MAX_DS };
    gse_outputs_t o;
    gse_cmd_t c; memset(&c, 0, sizeof c);

    /* 1. never connected -> safe state */
    gse_failsafe(NULL, 0xFFFF, &cfg, &o);
    assert(o.comms_loss && !o.armed && o.fill == 0 && o.veh == 1 && o.prg == 0 && o.ign == 0);

    /* 2. fresh command, not armed, igniter requested -> inhibited */
    c.fill = 1; c.prg = 1; c.veh = 0; c.ign = 1; c.arm_code = 0;
    gse_failsafe(&c, 100, &cfg, &o);
    assert(!o.comms_loss && !o.armed && o.fill == 1 && o.prg == 1 && o.veh == 0 && o.ign == 0);

    /* 3. armed + fresh + ign requested -> fires */
    c.arm_code = GSE_ARM_CODE;
    gse_failsafe(&c, 100, &cfg, &o);
    assert(o.armed && o.ign == 1 && o.ign_src == GSE_IGN_SRC_CMD);

    /* 4. arm code removed -> immediate inhibit, valves follow command */
    c.arm_code = 0;
    gse_failsafe(&c, 100, &cfg, &o);
    assert(!o.armed && o.ign == 0 && o.fill == 1);

    /* 5. watchdog expiry on the very last frame edge */
    c.arm_code = GSE_ARM_CODE;
    gse_failsafe(&c, cfg.wdg_ds * 100, &cfg, &o);      /* exactly at limit: ok */
    assert(!o.comms_loss);
    gse_failsafe(&c, (uint16_t)(cfg.wdg_ds * 100 + 1), &cfg, &o);  /* 1 ms late */
    assert(o.comms_loss);
    assert(o.fill == 0 && o.prg == 0 && o.veh == 1 && o.ign == 0);
    printf("  ok  failsafe matrix\n");
}

static void test_stream_split(void)
{
    /* Simulate 500 random-ish CMD frames concatenated with noise bytes and
       verify every intact frame is delivered. */
    gse_parser_t p; gse_parser_init(&p); g_frames = 0;
    uint32_t rng = 12345;
    for (int f = 0; f < 500; ++f) {
        gse_cmd_t c; memset(&c, 0, sizeof c);
        c.tx_ms = (uint16_t)f; c.fill = (uint8_t)(f & 1);
        uint8_t buf[GSE_MAX_FRAME];
        size_t n = gse_encode_cmd((uint8_t)f, &c, buf);
        rng = rng * 1103515245u + 12345u;
        int noise = (int)((rng >> 16) % 3);        /* 0..2 junk bytes */
        for (int k = 0; k < noise; ++k) {
            rng = rng * 1103515245u + 12345u;
            gse_parser_push(&p, (uint8_t)(rng >> 24), on_frame, NULL);
        }
        for (size_t i = 0; i < n; ++i) gse_parser_push(&p, buf[i], on_frame, NULL);
    }
    assert(g_frames == 500);
    printf("  ok  500-frame noisy stream (%lu false frames rejected)\n",
           (unsigned long)p.frames_bad);
}

int main(void)
{
    printf("gse_protocol tests\n");
    test_crc_known_vector();
    test_roundtrip_cmd();
    test_roundtrip_state();
    test_corruption_recovery();
    test_failsafe_matrix();
    test_stream_split();
    printf("ALL PASS\n");
    return 0;
}
