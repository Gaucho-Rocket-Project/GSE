/*
 * pad_node.cpp  --  Mojave Sphinx GSE, launch-pad node
 * -------------------------------------------------------------------------
 * Radio in : DX-LR41-24T12S (UART transparent pipe, 2.4 GHz LoRa/FLRC)
 * Outputs  : 3x servo PWM (FILL / VEHICLE / PURGE) + 1x igniter relay
 *
 * Safety model (why this is not "just an ESP32 driving servos")
 *   1. Outputs are recomputed every 20 ms from (last accepted command, local
 *      clock).  The laptop link can lie, hang or vanish; the local clock cannot.
 *   2. Watchdog: no valid CMD for > cfg.wdg_ds (default 1.0 s) => outputs snap
 *      to FILL CLOSED / PURGE CLOSED / VEHICLE OPEN / IGN OFF.  This is the
 *      same safe set the stock FS-iA10B failsafes are configured for in
 *      HCR-5100 OP 8 ("Configure receiver failsafes").
 *   3. Arming: the igniter relay can only close while (a) the CMD frame carries
 *      arm_code == 0xA5, (b) the link is fresh, and (c) the on-time limiter
 *      (default 0.5 s, hard) has not expired.  Releasing the arm control, or
 *      losing one frame, drops the arm latch in the same 20 ms tick.
 *   4. Hardware keep-alive: loop() toggles PIN_HB at 50 Hz.  An RC network
 *      (see hardware/wiring.md) turns loss of that square wave into a power
 *      cut on the igniter relay coil only -- protection against a firmware
 *      hang, independent of software.
 *   5. Boot state is safe and is re-asserted before the radio is configured.
 */
#include <Arduino.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "gse_protocol.h"
#include "dxlr41.h"
#include "pad_diag.h"

#ifdef GSE_ROLE_TX
#error "pad_node.cpp must be built with -DGSE_ROLE_PAD"
#endif

static_assert(GSE_MAX_FRAME + 3 <= 64, "frame fits the 64-byte module chunk");

/* ------------------------------------------------------------------ objects */
HardwareSerial LoraSerial(1);
DxLr41 lora(LoraSerial, PIN_LORA_AUX, PIN_LORA_M0, PIN_LORA_M1);
static gse_parser_t parser;

/* -------------------------------------------------------------------- state */
static gse_cfg_t cfg       = { GSE_WDG_DS_DEFAULT, GSE_IGN_MAX_DS_DEFAULT };
static gse_cmd_t lastCmd;
static bool      haveCmd   = false;
static uint32_t  lastCmdMs = 0;
static uint16_t  rxCount   = 0;
static uint16_t  crcErr    = 0;
static uint8_t   padState  = GSE_PAD_WAIT_HELLO;
static uint8_t   cfgSeqEcho = 0;
static uint8_t   travel[3] = { SERVO_DEFAULT_TRAVEL, SERVO_DEFAULT_TRAVEL,
                               SERVO_DEFAULT_TRAVEL };
static bool      allowPurge = (GSE_ALLOW_PURGE_DEFAULT != 0);
static uint32_t  ignOnSince = 0;
static bool      ignOn      = false;
static bool      ignSpent   = false;   /* set by the on-time limit, cleared
                                          only when the request is released */
static uint32_t  lastPulseMs = 0, lastStateMs = 0, t0Ms = 0;
static uint8_t   seqOut = 0;
static char      cfgFault[40] = {0};
static uint32_t  lastDiagMs = 0, lastByteMs = 0, lastLedMs = 0;
static bool      phase = false;
static uint32_t  rxBytes = 0, sfFrames = 0;   /* anything at all off the module */

/* ------------------------------------------------------------------ helpers */
/* 50 Hz, 12-bit LEDC: period 20 000 us == 4096 counts -> 0.2048 counts/us  */
#define US_TO_DUTY(us) ((uint32_t)(((uint32_t)(us) * 4096u) / 20000u))

static inline void servoUs(int idx, int state01, uint8_t travelDeg)
{
    static const uint8_t pins[3] = { PIN_SERVO_FILL, PIN_SERVO_VEH, PIN_SERVO_PRG };
    int deg  = constrain((int)travelDeg, SERVO_MIN_TRAVEL, SERVO_MAX_TRAVEL);
    // int span = (SERVO_MAX_US - SERVO_MIN_US) / 2;            /* 500us / 45deg */
    int span = SERVO_US_PER_45DEG;           /* scale no longer tied to the clamp */
    int us   = SERVO_NEUTRAL_US + (state01 ? span * deg / 45 : -span * deg / 45);
    us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pins[idx], US_TO_DUTY(us));          /* core 3.x: pin-keyed   */
#else
    (void)pins;
    ledcWrite(idx, US_TO_DUTY(us));                /* core 2.x: channel     */
#endif
}

static void driveOutputs(const gse_outputs_t &o)
{
    servoUs(0, o.fill, travel[0]);
    servoUs(1, o.veh,  travel[1]);
    servoUs(2, allowPurge ? o.prg : 0, travel[2]);              /* never purge */
}

/* Outgoing frames carry the fixed-point prefix: dst address (2 B) + dst
 * channel (1 B).  The sending module strips it and the receiving module never
 * passes it on, so the two nodes exchange bare 7E-framed packets. */
static void sendFrame(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t frame[GSE_MAX_FRAME];
    uint8_t pkt[GSE_MAX_FRAME + 4];
    size_t n = gse_encode(type, seqOut++, payload, len, frame);
    size_t m = lora.buildFixedPoint(pkt, GSE_ADDR_TX, GSE_CH_PEER(0), frame, n);
    lora.write(pkt, m);
}

static void sendEvent(uint8_t code, uint8_t arg)
{
    gse_event_t e = { code, arg, 0 };
    sendFrame(GSE_T_EVENT, &e, sizeof e);
}

/* ------------------------------------------------------------- frame handler */
static void onFrame(uint8_t type, uint8_t, const uint8_t *pl, uint8_t len, void *)
{
    switch (type) {
    case GSE_T_CMD:
        if (len != sizeof(gse_cmd_t)) { crcErr++; return; }
        memcpy(&lastCmd, pl, sizeof lastCmd);
        haveCmd   = true;
        lastCmdMs = millis();
        rxCount++;
        if (padState == GSE_PAD_WAIT_HELLO) padState = GSE_PAD_RUN;
        if (lastCmd.wdg_ds)
            cfg.wdg_ds = constrain(lastCmd.wdg_ds, GSE_WDG_DS_MIN, GSE_WDG_DS_MAX);
        if (lastCmd.ign_max_ds)
            cfg.ign_max_ds = constrain(lastCmd.ign_max_ds, GSE_IGN_MAX_DS_MIN,
                                       GSE_IGN_MAX_DS_MAX);
        break;

    case GSE_T_CFG: {
        if (len != sizeof(gse_cfgs_t)) { crcErr++; return; }
        gse_cfgs_t c; memcpy(&c, pl, sizeof c);
        travel[0] = constrain((int)c.travel_fill, SERVO_MIN_TRAVEL, SERVO_MAX_TRAVEL);
        travel[1] = constrain((int)c.travel_veh,  SERVO_MIN_TRAVEL, SERVO_MAX_TRAVEL);
        travel[2] = constrain((int)c.travel_prg,  SERVO_MIN_TRAVEL, SERVO_MAX_TRAVEL);
        cfg.wdg_ds = constrain(c.wdg_ds ? c.wdg_ds : GSE_WDG_DS_DEFAULT,
                               GSE_WDG_DS_MIN, GSE_WDG_DS_MAX);
        cfg.ign_max_ds = constrain(c.ign_max_ds ? c.ign_max_ds : GSE_IGN_MAX_DS_DEFAULT,
                                   GSE_IGN_MAX_DS_MIN, GSE_IGN_MAX_DS_MAX);
        allowPurge = c.allow_purge != 0;
        cfgSeqEcho = c.cfg_seq;
        sendEvent(GSE_EV_CFG_CHANGED, cfgSeqEcho);
        break;
    }

    case GSE_T_HELLO:
        if (len >= 3 && pl[0] != GSE_NODE_PAD) {
            if (padState == GSE_PAD_WAIT_HELLO) padState = GSE_PAD_RUN;
            uint8_t pl2[3] = { GSE_NODE_PAD, GSE_FW_MAJOR, GSE_FW_MINOR };
            sendFrame(GSE_T_HELLO, pl2, 3);
        }
        break;
    default: break;
    }
}

/* ------------------------------------------------------------------ telemetry */
static void sendState(uint16_t age, const gse_outputs_t &o)
{
    gse_state_t s; memset(&s, 0, sizeof s);
    s.flags      = (uint8_t)((o.comms_loss ? 0 : GSE_ST_LINK_OK) |
                             (haveCmd ? 0 : GSE_ST_LATCHED) |
                             (o.armed ? GSE_ST_ARMED : 0) |
                             (o.ign ? GSE_ST_IGN_ON : 0) |
                             (o.comms_loss ? GSE_ST_COMMS_LOSS : 0) |
                             (padState == GSE_PAD_RUN ? GSE_ST_HELLO_OK : 0));
    s.out_fill   = o.fill;  s.out_veh = o.veh; s.out_prg = o.prg; s.out_ign = o.ign;
    s.ign_src    = o.ign_src;
    s.age_ms     = age;
    s.rx_count   = rxCount;
    s.crc_err    = crcErr;
    s.rssi_dbm   = (int8_t)0x7F;                 /* not enabled on the module */
    s.pad_state  = padState;
    s.uptime_s   = (uint16_t)((millis() - t0Ms) / 1000u);
    s.wdg_ds     = cfg.wdg_ds;
    s.ign_max_ds = cfg.ign_max_ds;
    s.fw_major   = GSE_FW_MAJOR;  s.fw_minor = GSE_FW_MINOR;
    s.travel_fill = travel[0]; s.travel_veh = travel[1]; s.travel_prg = travel[2];
    s.allow_purge = allowPurge ? 1 : 0;
    sendFrame(GSE_T_STATE, &s, sizeof s);
}

/* ---------------------------------------------------------------- ignition */
/*
 * Igniter sequencing (deliberately "one-shot per press"):
 *   request 0                -> relay off, spent latch cleared
 *   request 1, not spent     -> relay on, hard cut at cfg.ign_max_ds
 *   request 1, spent         -> relay stays off until the operator releases
 *                               and re-asserts (prevents a stuck command or a
 *                               held GUI button from pulsing the e-match)
 */
static void serviceIgniter(bool request, uint32_t now)
{
    if (!request) {
        ignSpent = false;
        if (ignOn) { ignOn = false; sendEvent(GSE_EV_IGN_OFF, 0); }
        digitalWrite(PIN_RELAY_IGN, LOW);
        return;
    }
    if (ignSpent) { digitalWrite(PIN_RELAY_IGN, LOW); return; }
    if (!ignOn) {
        ignOn = true; ignOnSince = now;
        sendEvent(GSE_EV_IGN_ON, 0);
        digitalWrite(PIN_RELAY_IGN, HIGH);
        return;
    }
    if ((now - ignOnSince) > (uint32_t)cfg.ign_max_ds * 100u) {
        ignOn = false; ignSpent = true;
        digitalWrite(PIN_RELAY_IGN, LOW);              /* hard cut            */
        sendEvent(GSE_EV_IGN_OFF, 1);
        return;
    }
    digitalWrite(PIN_RELAY_IGN, HIGH);
}

/* ---------------------------------------------------------------------- main */
void setup()
{
    t0Ms = millis();
    pinMode(PIN_RELAY_IGN, OUTPUT);
    digitalWrite(PIN_RELAY_IGN, LOW);
    pinMode(PIN_HB, OUTPUT);

    const int pins[3] = { PIN_SERVO_FILL, PIN_SERVO_VEH, PIN_SERVO_PRG };
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    for (int i = 0; i < 3; ++i) ledcAttach(pins[i], 50, 12);
#else
    for (int ch = 0; ch < 3; ++ch) { ledcSetup(ch, 50, 12); ledcAttachPin(pins[ch], ch); }
#endif

    gse_outputs_t safe;
    gse_cfg_t bootCfg = cfg;
    gse_failsafe(nullptr, 0xFFFF, &bootCfg, &safe);
    driveOutputs(safe);                                /* safe before radio  */

    diag::begin();
    diag::line("PAD boot fw %u.%u  pins FILL=%d VEH=%d PRG=%d IGN=%d HB=%d",
               GSE_FW_MAJOR, GSE_FW_MINOR, PIN_SERVO_FILL, PIN_SERVO_VEH,
               PIN_SERVO_PRG, PIN_RELAY_IGN, PIN_HB);
    diag::led(0, 0, 40);                       /* blue: booting            */

    lora.begin(DXLR41_BAUD);
    diag::line("PAD radio provisioning: MODE1 LEVEL%u CH%02X KEY%u MAC%02X,%02X",
               GSE_LEVEL_DEFAULT, GSE_CH_SELF(0), GSE_KEY_WORD,
               GSE_ADDR_PAD >> 8, GSE_ADDR_PAD & 0xFF);
    String fault = lora.configure(GSE_LEVEL_DEFAULT, GSE_CH_SELF(0), GSE_KEY_WORD,
                                  true, GSE_ADDR_PAD, GSE_ADDR_TX);
    if (fault.length()) {
        strncpy(cfgFault, fault.c_str(), sizeof(cfgFault) - 1);
        padState = GSE_PAD_LATCHED;
        diag::line("PAD RADIO FAULT: %s", cfgFault);
        diag::line("PAD outputs held SAFE; fix the module wiring/baud and reset.");
        diag::led(40, 0, 0);                   /* red: radio fault         */
    } else {
        padState = GSE_PAD_WAIT_HELLO;
        diag::line("PAD RADIO OK  (waiting for the TX node)");
        diag::led(40, 20, 0);                  /* amber: no link yet       */
    }
    /* Software watchdog: a wedged loop must not leave a valve commanded open
     * forever.  Reboot -> setup() re-asserts the safe outputs.                */
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    esp_task_wdt_config_t twdt = { .timeout_ms = 2000, .idle_core_mask = 0,
                                   .trigger_panic = true };
    esp_task_wdt_reconfigure(&twdt);
    esp_task_wdt_add(NULL);
#elif defined(ESP_ARDUINO_VERSION_MAJOR)
    esp_task_wdt_init(2, true);
    esp_task_wdt_add(NULL);
#endif
    sendEvent(GSE_EV_BOOT, cfgFault[0] ? 1 : 0);
}

void loop()
{
#if defined(ESP_ARDUINO_VERSION_MAJOR)
    esp_task_wdt_reset();
#endif
    uint32_t now = millis();

    /* 1. drain the radio --------------------------------------------------
     *    Every byte goes to the parser.  In fixed-point mode the *sending*
     *    module strips the 3-byte address/channel prefix, so what arrives is
     *    a clean 7E|ver|type|... frame; if a module revision ever leaks the
     *    prefix through, the parser's resync (0x7E followed by a valid
     *    version byte, see tests/test_protocol.c) skips it.  Either way no
     *    data byte is ever dropped by us, which is the failure this used to
     *    have: a `skipPrefix` counter that nothing ever set, sitting in the
     *    receive path and reading as though it worked.                       */
    while (lora.available()) {
        uint8_t b = (uint8_t)lora.read();
        rxBytes++;
        lastByteMs = now;
        if (b == GSE_SOF) sfFrames++;
        gse_parser_push(&parser, b, onFrame, nullptr);
    }

    /* 2. recompute the safe truth ---------------------------------------- */
    uint16_t age = haveCmd ? (uint16_t)min<uint32_t>(now - lastCmdMs, 0xFFFE) : 0xFFFF;
    gse_outputs_t o;
    gse_failsafe(haveCmd ? &lastCmd : nullptr, age, &cfg, &o);

    /* 3. 50 Hz output refresh + hardware keep-alive ----------------------- */
    if (now - lastPulseMs >= SERVO_REFRESH_MS) {
        lastPulseMs = now;
        driveOutputs(o);
        serviceIgniter(o.ign != 0, now);
        digitalWrite(PIN_HB, !digitalRead(PIN_HB));       /* 25 Hz square */

        /* status LED: the only channel that works when USB is broken */
        if (ignOn)               diag::led(40, 0, 50);   /* purple          */
        else if (cfgFault[0])    diag::led(40, 0, 0);    /* red             */
        else if (o.comms_loss)   diag::led((phase ? 40 : 0), 12, 0);
        else                     diag::led(0, (phase ? 40 : 0), 10);
        if (now - lastLedMs >= 500) { lastLedMs = now; phase = !phase; }
    }

    /* 4. edge events ------------------------------------------------------ */
    static uint8_t lastLoss = 0;
    if ((uint8_t)o.comms_loss != lastLoss) {
        sendEvent(o.comms_loss ? GSE_EV_COMMS_LOSS : GSE_EV_HELLO_OK, (uint8_t)age);
        lastLoss = (uint8_t)o.comms_loss;
    }

    /* 5. telemetry at 10 Hz ----------------------------------------------- */
    if (now - lastStateMs >= GSE_STATE_PERIOD_MS) {
        lastStateMs = now;
        sendState(age, o);
    }

    /* 6. diagnosis --------------------------------------------------------
     *    Printed once a second for the first ten seconds (so a terminal shows
     *    something immediately when the pad is powered) and every 5 s after
     *    that.  At 115200 one line is ~3 ms, far inside the 20 ms control
     *    tick, and it never blocks on a full USB buffer for long: Serial
     *    drops rather than stalls when nothing is reading the port.         */
    if (now - lastDiagMs >= (now < 10000u ? 1000u : 5000u)) {
        lastDiagMs = now;
        diag::line("PAD st=%u valid=%u bad=%u bytes=%lu sof=%lu age=%u %s",
                   padState, (unsigned)rxCount, (unsigned)crcErr,
                   (unsigned long)rxBytes, (unsigned long)sfFrames,
                   (unsigned)age,
                   cfgFault[0] ? cfgFault
                               : (o.comms_loss ? "NO-LINK (safe)"
                                               : "LINK OK"));
        if (!cfgFault[0] && rxBytes == 0 && now > 3000u)
            diag::line("PAD nothing on the radio UART: check TXD/RXD cross, "
                       "3V3, and that both modules are Level%u Ch%02X",
                       GSE_LEVEL_DEFAULT, GSE_CH_SELF(0));
    }
}
