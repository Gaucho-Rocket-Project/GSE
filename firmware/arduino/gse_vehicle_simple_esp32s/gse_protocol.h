/*
 * gse_protocol.h  --  Mojave Sphinx / HCR-5100 GSE radio link protocol
 * -------------------------------------------------------------------------
 * Shared, header-only wire protocol used by:
 *      - tx-node  (ESP32-S3 #1, USB-C to laptop, drives DX-LR41 -> air)
 *      - pad-node (ESP32-S3 #2, drives 3 servo PWM channels + igniter relay)
 *      - tools/   (host-side unit tests, simulation)
 *      - software/gse_control/protocol.py  (byte-for-byte mirror for the GUI)
 *
 * FRAME (little-endian fields, fixed header):
 *   +-----+-----+------+-----+-----+---------------+-------+
 *   | 7E  | ver | type | seq | len | payload[len]  | crc16 |
 *   +-----+-----+------+-----+-----+---------------+-------+
 *   crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over ver..payload
 *   A 7E byte inside the CRC field is impossible to exclude, so 7E is treated
 *   only as a *resynchronisation* hint: any byte that follows a valid-CRC
 *   frame is parsed as a new frame start only if it is 7E.  A parser that
 *   loses sync simply drops one byte at a time.  At 20 Hz with 26-byte frames
 *   on a dedicated 2.4 GHz link this is sufficient; see docs 4.6.
 *
 * PAYLOAD UNITS
 *   fill / veh / prg : 0 = valve CLOSED, 1 = valve OPEN   (switch position)
 *   ign              : 0 = igniter relay OFF, 1 = OFF is *requested*, the pad
 *                      node decides (see GSE_FAILSAFE section below)
 *
 * FAILSAFE CONTRACT (identical in firmware, host test and GUI):
 *   1. Pad node latches LATCHED state if it has never received a valid CMD.
 *   2. age = ms since last valid CMD.  If age > watchdog -> COMMS LOSS.
 *   3. On COMMS LOSS the pad forces:
 *          FILL  = CLOSED,  PURGE = CLOSED,  VEHICLE = OPEN,  IGNITER = OFF
 *      (vehicle valves open = safe: dumps propellants; igniter off = safe)
 *   4. Arming (igniter permitted to fire) requires:
 *          arm_code == 0xA5 in *every* accepted CMD (i.e. the operator is
 *          actively holding the arm control)  AND
 *          link fresh (not latched) AND watchdog armed.
 *      If a single CMD arrives without the arm code, the pad immediately
 *      drops the arm latch and the igniter is inhibited.
 *   5. Igniter on-time is hard-limited by ign_max_ds (default 0.5 s).
 *
 * Copyright: GPL-3.0, same licence as HCR-5100.
 */
#ifndef GSE_PROTOCOL_H
#define GSE_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- constants */
#define GSE_PROTO_VERSION     1u

#define GSE_SOF               0x7Eu
#define GSE_MAX_PAYLOAD       32u
#define GSE_MAX_FRAME         (5u + GSE_MAX_PAYLOAD + 2u)

#define GSE_ARM_CODE          0xA5u

/* frame types */
#define GSE_T_HELLO           0x01u   /* tx -> pad, also pad -> tx on boot      */
#define GSE_T_CMD             0x10u   /* tx -> pad, 20 Hz                       */
#define GSE_T_STATE           0x11u   /* pad -> tx, 10 Hz                       */
#define GSE_T_EVENT           0x12u   /* pad -> tx, on change / fault           */
#define GSE_T_CFG             0x13u   /* tx -> pad, on change; pad echoes in STATE */
#define GSE_T_ACK             0x81u   /* reserved                               */

/* defaults, duplicated in config.h of each node */
#define GSE_DEF_WDG_DS        10u     /* 1.0 s   link watchdog                  */
#define GSE_DEF_IGN_MAX_DS    5u      /* 0.5 s   igniter on-time limit          */
#define GSE_CFG_LIMIT_WDG_DS  30u     /* clamp 0.3 .. 3.0 s                     */
#define GSE_CFG_LIMIT_IGN_DS  20u     /* clamp 0.1 .. 2.0 s                     */

/* command flag bits */
#define GSE_CF_REQ_ACK        0x01u

/* state flag bits */
#define GSE_ST_LINK_OK        0x01u
#define GSE_ST_LATCHED        0x02u   /* no valid command yet / fault latched   */
#define GSE_ST_ARMED          0x04u   /* arm interlock satisfied                */
#define GSE_ST_IGN_ON         0x08u   /* relay actually energised               */
#define GSE_ST_COMMS_LOSS     0x10u   /* watchdog has expired                   */
#define GSE_ST_HELLO_OK       0x20u   /* handshake complete                     */

/* igniter inhibit-source bits, packed into gse_state_t.ign_src */
#define GSE_IGN_SRC_CMD       0x01u   /* operator commanded ignition            */
#define GSE_IGN_SRC_WDG       0x02u   /* watchdog fired the igniter (option)    */

/* pad node run state */
#define GSE_PAD_BOOT          0u
#define GSE_PAD_WAIT_HELLO    1u
#define GSE_PAD_RUN           2u
#define GSE_PAD_LATCHED       3u

/* ------------------------------------------------------------------- CRC16 */
static inline uint16_t gse_crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < n; ++i) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

/* --------------------------------------------------------------- structures */
#pragma pack(push, 1)

typedef struct {                    /* 12 bytes, GSE_T_CMD                    */
    uint8_t arm_code;               /* GSE_ARM_CODE while arming, else 0      */
    uint8_t fill;                   /* 0/1                                    */
    uint8_t veh;                    /* 0/1                                    */
    uint8_t prg;                    /* 0/1                                    */
    uint8_t ign;                    /* 0/1 (request only)                     */
    uint8_t wdg_ds;                 /* 0 = default                            */
    uint8_t ign_max_ds;             /* 0 = default                            */
    uint8_t flags;                  /* GSE_CF_*                               */
    uint16_t tx_ms;                 /* sender millis, wraps, for staleness    */
    uint8_t ign_wdg;                /* 1 = allow watchdog to fire igniter     */
    uint8_t reserved;
} gse_cmd_t;

typedef struct {                    /* 24 bytes, GSE_T_STATE                  */
    uint8_t  flags;                 /* GSE_ST_*                               */
    uint8_t  out_fill;              /* physical output state                  */
    uint8_t  out_veh;
    uint8_t  out_prg;
    uint8_t  out_ign;
    uint8_t  ign_src;               /* GSE_IGN_SRC_*                          */
    uint16_t age_ms;                /* 0xFFFF = never                         */
    uint16_t rx_count;              /* valid CMDs, lower 16 bits              */
    uint16_t crc_err;               /* bad CRCs, lower 16 bits                */
    int8_t   rssi_dbm;              /* 0x7F = unknown                         */
    uint8_t  pad_state;             /* GSE_PAD_*                              */
    uint16_t uptime_s;              /* lower 16                               */
    uint8_t  wdg_ds;                /* watchdog in force                      */
    uint8_t  ign_max_ds;            /* igniter limit in force                 */
    uint8_t  fw_major;
    uint8_t  fw_minor;
    uint8_t  travel_fill;           /* servo throw, deg each side of neutral  */
    uint8_t  travel_veh;
    uint8_t  travel_prg;
    uint8_t  allow_purge;
} gse_state_t;

typedef struct {                    /* 8 bytes, GSE_T_CFG                     */
    uint8_t  travel_fill;
    uint8_t  travel_veh;
    uint8_t  travel_prg;
    uint8_t  wdg_ds;
    uint8_t  ign_max_ds;
    uint8_t  allow_purge;
    uint8_t  cfg_seq;               /* incrementing, echoed in the EVENT      */
    uint8_t  reserved;
} gse_cfgs_t;

typedef struct {                    /* 3 bytes, GSE_T_EVENT                   */
    uint8_t  code;                  /* see GSE_EV_*                           */
    uint8_t  arg;
    uint8_t  pad;
} gse_event_t;

#pragma pack(pop)

/* event codes */
#define GSE_EV_BOOT           1u
#define GSE_EV_HELLO_OK       2u
#define GSE_EV_COMMS_LOSS     3u
#define GSE_EV_ARM            4u
#define GSE_EV_DISARM         5u
#define GSE_EV_IGN_ON         6u
#define GSE_EV_IGN_OFF        7u
#define GSE_EV_CFG_CHANGED    8u
#define GSE_EV_BAD_CMD        9u

/* ------------------------------------------------------------ encode/decode */
static inline size_t gse_encode(uint8_t type, uint8_t seq,
                                const void *payload, uint8_t len,
                                uint8_t *out)
{
    if (len > GSE_MAX_PAYLOAD) return 0;
    out[0] = GSE_SOF;
    out[1] = (uint8_t)GSE_PROTO_VERSION;
    out[2] = type;
    out[3] = seq;
    out[4] = len;
    if (len && payload) memcpy(&out[5], payload, len);
    uint16_t crc = gse_crc16(&out[1], (size_t)4u + len);   /* ver..payload */
    out[5 + len] = (uint8_t)(crc & 0xFF);
    out[6 + len] = (uint8_t)(crc >> 8);
    return (size_t)7u + len;
}

typedef void (*gse_frame_cb)(uint8_t type, uint8_t seq,
                             const uint8_t *payload, uint8_t len, void *ctx);

typedef struct {
    uint8_t buf[GSE_MAX_FRAME];
    uint8_t len;
    uint32_t frames_ok;
    uint32_t frames_bad;
} gse_parser_t;

static inline void gse_parser_init(gse_parser_t *p)
{
    p->len = 0; p->frames_ok = 0; p->frames_bad = 0;
}

/* Drop the first buffered byte and resynchronise.  Used whenever a frame
   candidate fails a sanity check; the stream may hold a valid SOF later. */
static inline void gse_parser_drop(gse_parser_t *p)
{
    if (p->len) { memmove(p->buf, p->buf + 1, (size_t)(p->len - 1)); p->len--; }
    p->frames_bad++;
    /* discard leading junk up to the next SOF, but never the byte we just
       unshifted if it is itself an SOF */
    while (p->len > 0 && p->buf[0] != GSE_SOF) {
        memmove(p->buf, p->buf + 1, (size_t)(p->len - 1)); p->len--;
    }
}

/* Feed one byte; returns 1 when a frame was delivered. */
static inline int gse_parser_push(gse_parser_t *p, uint8_t b,
                                  gse_frame_cb cb, void *ctx)
{
    if (p->len == 0) {                          /* hunting for SOF */
        if (b != GSE_SOF) return 0;
        p->buf[0] = b; p->len = 1; return 0;
    }
    if (p->len >= GSE_MAX_FRAME) { p->len = 0; return 0; }
    p->buf[p->len++] = b;

    for (;;) {
        if (p->len == 0) return 0;
        if (p->buf[0] != GSE_SOF) { gse_parser_drop(p); continue; }
        /* version byte is checked as early as possible so that a stray SOF
           immediately in front of a real frame costs one byte, not a frame */
        if (p->len >= 2 && p->buf[1] != GSE_PROTO_VERSION) { gse_parser_drop(p); continue; }
        if (p->len < 5) return 0;
        uint8_t len = p->buf[4];
        if (len > GSE_MAX_PAYLOAD) { gse_parser_drop(p); continue; }
        size_t need = (size_t)7u + len;
        if (p->len < need) return 0;
        if (p->buf[1] != GSE_PROTO_VERSION) { gse_parser_drop(p); continue; }
        uint16_t rx_crc = (uint16_t)((uint16_t)p->buf[5 + len] |
                                     ((uint16_t)p->buf[6 + len] << 8));
        uint16_t calc = gse_crc16(&p->buf[1], (size_t)4u + len);
        if (rx_crc != calc) { gse_parser_drop(p); continue; }
        p->frames_ok++;
        if (cb) cb(p->buf[2], p->buf[3], &p->buf[5], len, ctx);
        p->len = 0;
        return 1;
    }
}

/* typed helpers ---------------------------------------------------------- */
static inline size_t gse_encode_cmd(uint8_t seq, const gse_cmd_t *c, uint8_t *out)
{ return gse_encode(GSE_T_CMD, seq, c, (uint8_t)sizeof(*c), out); }

static inline size_t gse_encode_state(uint8_t seq, const gse_state_t *s, uint8_t *out)
{ return gse_encode(GSE_T_STATE, seq, s, (uint8_t)sizeof(*s), out); }

static inline size_t gse_encode_event(uint8_t seq, const gse_event_t *e, uint8_t *out)
{ return gse_encode(GSE_T_EVENT, seq, e, (uint8_t)sizeof(*e), out); }

static inline size_t gse_encode_cfg(uint8_t seq, const gse_cfgs_t *c, uint8_t *out)
{ return gse_encode(GSE_T_CFG, seq, c, (uint8_t)sizeof(*c), out); }

static inline size_t gse_encode_hello(uint8_t seq, uint8_t node, uint8_t major,
                                      uint8_t minor, uint8_t *out)
{
    uint8_t pl[3] = { node, major, minor };
    return gse_encode(GSE_T_HELLO, seq, pl, 3, out);
}

/* ------------------------------------------------------- failsafe decision  */
typedef struct {
    uint8_t   wdg_ds;        /* in force */
    uint8_t   ign_max_ds;    /* in force */
} gse_cfg_t;

typedef struct {
    uint8_t fill, veh, prg, ign;     /* outputs the pad must drive */
    uint8_t armed;                   /* arm interlock satisfied    */
    uint8_t comms_loss;              /* watchdog expired           */
    uint8_t ign_src;                 /* GSE_IGN_SRC_*              */
} gse_outputs_t;

static inline uint8_t gse_clamp_u8(uint8_t v, uint8_t lo, uint8_t hi)
{ return v < lo ? lo : (v > hi ? hi : v); }

/*
 * Pure function: given the last accepted command (or NULL), how long ago it
 * arrived (0xFFFF = never), and the configuration in force, return the exact
 * outputs the pad node must drive.  Identical logic is mirrored in
 * software/gse_control/protocol.py -- keep the two in step.
 */
static inline void gse_failsafe(const gse_cmd_t *last, uint16_t age_ms,
                                const gse_cfg_t *cfg, gse_outputs_t *o)
{
    memset(o, 0, sizeof(*o));
    uint8_t armed = (uint8_t)(last && last->arm_code == GSE_ARM_CODE);
    uint32_t wdg_ms = (uint32_t)cfg->wdg_ds * 100u;
    uint8_t fresh = (uint8_t)(last != 0 && age_ms != 0xFFFFu &&
                              (uint32_t)age_ms <= wdg_ms);
    o->comms_loss = (uint8_t)(!fresh);
    o->armed      = (uint8_t)(armed && fresh);

    if (!fresh) {
        /* COMMS LOSS / NEVER-CONNECTED: everything safe */
        o->fill = 0; o->prg = 0; o->veh = 1; o->ign = 0;
        o->ign_src = 0;
        return;
    }

    /* normal pass-through, igniter gated by the arm interlock and the
       optional watchdog-triggered ignition feature                     */
    o->fill = last->fill ? 1u : 0u;
    o->veh  = last->veh  ? 1u : 0u;
    o->prg  = last->prg  ? 1u : 0u;

    uint8_t wdg_ign = (uint8_t)(last->ign_wdg && o->comms_loss); /* never */
    uint8_t cmd_ign = (uint8_t)(last->ign && armed && fresh);
    o->ign     = (uint8_t)((cmd_ign || wdg_ign) ? 1u : 0u);
    o->ign_src = (uint8_t)((cmd_ign ? GSE_IGN_SRC_CMD : 0u) |
                           (wdg_ign ? GSE_IGN_SRC_WDG : 0u));
}

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* GSE_PROTOCOL_H */
