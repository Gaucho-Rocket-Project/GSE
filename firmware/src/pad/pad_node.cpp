/*
 * pad_node.cpp  --  Mojave Sphinx GSE, launch-pad node
 * -------------------------------------------------------------------------
 * Link in  : ESP-NOW by default, or DX-LR42/LR41 with GSE_LINK_LORA.
 *            Both carry the same protocol and use the same safety logic.
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
 *   4. The loop toggles PIN_HB every 20 ms for an independently designed
 *      edge-sensitive hardware watchdog. The old RC example is NOT a valid
 *      stuck-high detector. External fail-safe hardware must be independently
 *      reviewed and tested; see hardware/wiring.md.
 *   5. Boot state is safe and is re-asserted before the radio is configured.
 */
#include <Arduino.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "gse_protocol.h"
#include "pad_diag.h"


#ifdef GSE_ROLE_TX
#error "pad_node.cpp must be built with -DGSE_ROLE_PAD"
#endif

/* Every AT+HELP read-back the driver performs is echoed to the console.  This
 * is the "is the module actually what I think it is" line: MODE / LEVEL /
 * Frequency / MAC / SLEEP in the pad's own log, at the moment of provisioning,
 * whether or not the verification then passes.  Only the LoRa transport has a
 * module with a banner to echo, so it is compiled out of a ESP-NOW build, where it
 * would be dead code and an unused-function warning.
 *
 * Note: this must stay above the link object, which includes dxlr41.h.  A
 * clean-build (or restart) failure that only appears after a successful build
 * is this define arriving too late, because the .cpp that last touched the
 * header still has the old object file. */
#if defined(GSE_LINK_LORA)
#define GSE_RADIO_TRACE(fmt, text) diag::blob("RADIO", (text))
#endif
#include "link.h"

static_assert(GSE_MAX_FRAME + 3 <= 64, "frame fits the module chunk");

/* ------------------------------------------------------------------ objects
 * gseLink is the transport seam (src/link.h): LoRa module or ESP-NOW, chosen at
 * build time.  Everything else in this file is transport-agnostic -- it feeds
 * bytes from gseLink.read() to the parser and hands frames to gseLink.send().
 *
 * Why it is not called `link`: the real arduino-esp32 Arduino.h pulls in
 * <unistd.h> (via lwIP and IPAddress.h), which declares the POSIX
 * `int link(const char*, const char*)` at global scope.  A variable named
 * `link` at namespace scope is then a redeclaration clash, and every
 * `link.send()` becomes "request for member 'send' in 'link', which is of
 * non-class type 'int(const char*, const char*)'".  The host syntax check in
 * tests/ did not catch this until tests/esp32_stub/Arduino.h grew the same
 * POSIX declarations -- which it now has, on purpose. */
static GseLink gseLink;
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

/* ------------------------------------------------------------------- status
 * One place that maps "what is the pad doing" to "what the indicator shows",
 * because the two board variants have different indicators: a WS2812 that can
 * show a colour (ESP32-S3) and a single blue LED that cannot (ESP-32S).  The
 * colour build keeps its original colours; the single-LED build gets a blink
 * pattern instead.  Either way the state is readable without a serial port,
 * which is the point -- USB is often the thing that is broken.            */
enum : uint8_t { LED_BOOT, LED_RADIO_FAULT, LED_NO_LINK, LED_LINK_OK, LED_IGN_ON };

static void led_show(uint8_t mode, bool ph)
{
#if GSE_HAS_RGB_LED
    switch (mode) {
    case LED_RADIO_FAULT: diag::led(40, 0, 0);  break;   /* red             */
    case LED_NO_LINK:     diag::led(ph ? 40 : 0, 12, 0); break; /* amber    */
    case LED_LINK_OK:     diag::led(0, ph ? 40 : 0, 10); break; /* green    */
    case LED_IGN_ON:      diag::led(40, 0, 50); break;   /* purple          */
    default:              diag::led(0, 0, 40);  break;   /* blue: booting   */
    }
#else
    (void)ph;                    /* the pattern is generated inside led_simple */
    switch (mode) {
    case LED_RADIO_FAULT: diag::led_simple(1, 100); break;  /* 5 Hz fast    */
    case LED_NO_LINK:     diag::led_simple(1, 250); break;  /* 2 Hz         */
    case LED_LINK_OK:     diag::led_simple(1, 500); break;  /* 1 Hz         */
    case LED_IGN_ON:      diag::led_simple(1,  50); break;  /* 10 Hz urgent */
    default:              diag::led_simple(1,   0); break;  /* solid on     */
    }
#endif
}

/* ------------------------------------------------------------------ helpers */
/* 50 Hz, 12-bit LEDC: period 20 000 us == 4096 counts -> 0.2048 counts/us  */
#define US_TO_DUTY(us) ((uint32_t)(((uint32_t)(us) * 4096u) / 20000u))

static inline void servoUs(int idx, int state01, uint8_t travelDeg)
{
    static const uint8_t pins[3] = { PIN_SERVO_FILL, PIN_SERVO_VEH, PIN_SERVO_PRG };
    int deg  = constrain((int)travelDeg, SERVO_MIN_TRAVEL, SERVO_MAX_TRAVEL);
    int span = SERVO_US_PER_45DEG;            /* scale, not tied to the clamp */
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

/* Outgoing frames are bare 7E packets.  On LoRa the link adds the fixed-point
 * prefix -- dst address (2 B) + dst channel (1 B) -- which the sending module
 * strips and the receiving module never passes on; over ESP-NOW there is no such
 * prefix and the frame goes out as-is.  See link.h. */
static void sendFrame(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t frame[GSE_MAX_FRAME];
    size_t n = gse_encode(type, seqOut++, payload, len, frame);
    gseLink.send(GSE_ADDR_TX, GSE_CH_PEER(0), frame, n);
}

static void sendEvent(uint8_t code, uint8_t arg)
{
    gse_event_t e = { code, arg, 0 };
    sendFrame(GSE_T_EVENT, &e, sizeof e);
}

/* ------------------------------------------------------------- frame handler */
static void onFrame(uint8_t type, uint8_t, const uint8_t *pl, uint8_t len, void *)
{
    /* A failed transport setup is latched until reset; stray UART bytes or a
     * partial initialization must never revive outputs after a link fault. */
    if (cfgFault[0]) return;
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
                             (ignOn ? GSE_ST_IGN_ON : 0) |
                             (o.comms_loss ? GSE_ST_COMMS_LOSS : 0) |
                             (padState == GSE_PAD_RUN ? GSE_ST_HELLO_OK : 0));
    s.out_fill   = o.fill;  s.out_veh = o.veh; s.out_prg = allowPurge ? o.prg : 0; s.out_ign = ignOn ? 1 : 0;
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
    diag::led_pin_begin();                    /* plain LED board only      */

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
    diag::line("PAD boot fw %u.%u board=%s pins FILL=%d VEH=%d PRG=%d IGN=%d HB=%d",
               GSE_FW_MAJOR, GSE_FW_MINOR, GSE_BOARD_NAME,
               PIN_SERVO_FILL, PIN_SERVO_VEH, PIN_SERVO_PRG, PIN_RELAY_IGN, PIN_HB);
    led_show(LED_BOOT, false);                 /* blue / solid: booting    */

    /* The transport, named in the log before anything can fail.  A pad and a
     * TX node built with different GSE_LINK_* values cannot hear each other,
     * and this is the line that makes that obvious instead of mysterious.  */
    gseLink.begin();
    diag::line("PAD link: %s -- %s", gseLink.transport(), gseLink.describe().c_str());
#if defined(GSE_LINK_LORA)
    diag::line("PAD radio provisioning: MODE1 LEVEL%u CH%02X KEY%u MAC%02X,%02X",
               GSE_LEVEL_DEFAULT, GSE_CH_SELF(0), GSE_KEY_WORD,
               GSE_ADDR_PAD >> 8, GSE_ADDR_PAD & 0xFF);
#endif
    String fault = gseLink.provision(GSE_LEVEL_DEFAULT, GSE_CH_SELF(0), GSE_KEY_WORD,
                                  GSE_ADDR_PAD, GSE_ADDR_TX);
    if (fault.length()) {
        strncpy(cfgFault, fault.c_str(), sizeof(cfgFault) - 1);
        padState = GSE_PAD_LATCHED;
        diag::line("PAD LINK FAULT: %s", cfgFault);
        diag::line("PAD outputs held SAFE; fix the %s link and reset.",
                   gseLink.transport());
        led_show(LED_RADIO_FAULT, false);      /* red / fast: radio fault  */
    } else {
        padState = GSE_PAD_WAIT_HELLO;
        diag::line("PAD LINK OK  (waiting for the TX node)");
        led_show(LED_NO_LINK, false);          /* amber / 2 Hz: no link    */
#if defined(GSE_LINK_LORA)
        /* M0/M1 are only a mode input when the module is in AT+SWITCH=1.
         * Say which scheme is in force -- with the module's OWN answer, which
         * the driver had to ask for separately (AT+HELP does not list SWITCH
         * in v2.0) -- so a floating-pin module is visible in the log instead
         * of looking like a silent radio. */
        diag::line("PAD radio mode select: SWITCH=%s; M0/M1 must be tied low if "
                   "the module ever runs SWITCH=1",
                   gseLink.switchMode() < 0 ? "? (no answer)" :
                   gseLink.switchMode() ? "1 (PINS - fault!)" : "0 (AT commands)");
#else
        diag::line("PAD ESP-NOW: %s; the TX node must be built with "
                   "-DGSE_LINK_ESPNOW on the same channel, and its MAC must be "
                   "the one in this build's config.h",
                   gseLink.linkState());
#endif
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

    /* 0. transport housekeeping ------------------------------------------
     *    Both transports currently have a non-blocking no-op poll().  */
    gseLink.poll();

    /* 1. drain the link ---------------------------------------------------
     *    Every byte goes to the parser.  In fixed-point mode the *sending*
     *    module strips the 3-byte address/channel prefix, so what arrives is
     *    a clean 7E|ver|type|... frame; if a module revision ever leaks the
     *    prefix through, the parser's resync (0x7E followed by a valid
     *    version byte, see tests/test_protocol.c) skips it.  Either way no
     *    data byte is deliberately skipped. ESP-NOW drops a whole datagram
     *    on receive-ring overflow rather than overwriting an existing frame. */
    while (gseLink.available()) {
        int r = gseLink.read();
        if (r < 0) break;
        uint8_t b = (uint8_t)r;
        rxBytes++;
        lastByteMs = now;
        if (b == GSE_SOF) sfFrames++;
        gse_parser_push(&parser, b, onFrame, nullptr);
    }

    /* 2. recompute the safe truth ---------------------------------------- */
    bool validCmd = haveCmd && !cfgFault[0];
    uint16_t age = validCmd ? (uint16_t)min<uint32_t>(now - lastCmdMs, 0xFFFE) : 0xFFFF;
    gse_outputs_t o;
    gse_failsafe(validCmd ? &lastCmd : nullptr, age, &cfg, &o);

    /* 3. 50 Hz output refresh + hardware keep-alive ----------------------- */
    if (now - lastPulseMs >= SERVO_REFRESH_MS) {
        lastPulseMs = now;
        driveOutputs(o);
        serviceIgniter(o.ign != 0, now);
        digitalWrite(PIN_HB, !digitalRead(PIN_HB));       /* 25 Hz square */

        /* status LED: the only channel that works when USB is broken */
        if (now - lastLedMs >= 500) { lastLedMs = now; phase = !phase; }
        if (ignOn)               led_show(LED_IGN_ON, phase);
        else if (cfgFault[0])    led_show(LED_RADIO_FAULT, phase);
        else if (o.comms_loss)   led_show(LED_NO_LINK, phase);
        else                     led_show(LED_LINK_OK, phase);
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
        if (!cfgFault[0] && rxBytes == 0 && now > 3000u) {
#if defined(GSE_LINK_LORA)
            diag::line("PAD nothing on the radio UART: check TXD/RXD cross, "
                       "3V3, and that both modules are Level%u Ch%02X",
                       GSE_LEVEL_DEFAULT, GSE_CH_SELF(0));
#else
            diag::line("PAD nothing over ESP-NOW (%s): is the TX node built "
                       "with -DGSE_LINK_ESPNOW, powered, on channel %d, and is "
                       "its MAC the one in GSE_NOW_TX_MAC?",
                       gseLink.linkState(), (int)GSE_NOW_CHANNEL);
#endif
        }
    }
}
