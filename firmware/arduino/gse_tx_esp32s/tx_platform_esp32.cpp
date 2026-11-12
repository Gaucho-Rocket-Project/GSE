/*
 * tx_platform_esp32.cpp  --  Arduino/ESP32 platform layer for the TX core
 * ---------------------------------------------------------------------------
 * Same behaviour as the original tx_node.cpp, now placed underneath the
 * portable core in firmware/src/common/gse_tx_core.c so that the STM32 and the
 * ESP32 run byte-identical bridging logic.
 *
 * Builds for both ESP32 board variants (see the board-variant block in
 * config.h and the envs in platformio.ini):
 *
 *   ESP32-S3 (env tx-node)          ESP-32S / WROOM-32 (env tx-node-esp32s)
 *   ------------------------------  ----------------------------------------
 *   D16 = module TXD -> UART1 RX    D16 = module TXD -> UART1 RX
 *   D17 = module RXD <- UART1 TX    D17 = module RXD <- UART1 TX
 *   D18 = module AUX                D18 = module AUX
 *   laptop on native USB-CDC        laptop on the CP2102/CH340 bridge, UART0
 *
 * Transport: ESP-NOW by default, or a LoRa module with -DGSE_LINK_LORA.
 * In that build the UART1 pins, the module and its AT provisioning are simply
 * absent -- everything above gse_hal.h is identical, which is what the seam in
 * src/link.h buys.  See docs/LINK_OPTIONS.md.
 *
 * The radio is on UART1 in both cases, so the ESP-32S's bridge on UART0 is
 * never shared with it.  GSE_HOST_IS_BRIDGE (config.h) says which one this
 * build has; the only consequence here is that a bridge board must keep
 * UART0's default pins (GPIO1/GPIO3) and must never be reconfigured, while the
 * S3 build has to make sure the CDC port is switched on at all.
 */
#include <Arduino.h>
#include <stdarg.h>

#include "config.h"
#include "gse_hal.h"
#include "gse_tx_core.h"

/* Same trick as the pad node: echo the driver's AT+HELP read-back to the
 * laptop console, prefixed like every other text line.  Only the LoRa build
 * has a module with an AT+HELP banner to echo -- on ESP-NOW this would be dead
 * code (and an unused-function warning), so it is compiled out. */
#if defined(GSE_LINK_LORA)
#define GSE_RADIO_TRACE(fmt, text) radioTrace(text)

static void radioTrace(const char *text)
{
    Serial.print("LOG RADIO ");
    for (const char *p = text; *p; ++p) {
        if (*p == '\n') Serial.print("\r\nLOG | ");
        else Serial.write((uint8_t)*p);
    }
    Serial.print("\r\n");
}
#endif

#include "link.h"

#ifdef GSE_ROLE_PAD
#error "tx_platform_esp32.cpp must be built with -DGSE_ROLE_TX"
#endif

#if !GSE_HOST_IS_BRIDGE && !defined(ARDUINO_USB_CDC_ON_BOOT)
#warning "ESP32-S3 TX build without ARDUINO_USB_CDC_ON_BOOT: the laptop may not see a port"
#endif

static bool radioOk = false;

/* The transport seam (src/link.h): LoRa module on UART1, or ESP-NOW.  The driver
 * needs the trace hook at *its* compile time, i.e. before the object exists --
 * GSE_RADIO_TRACE is defined above, the object here.
 *
 * Named gseLink, not `link`: <unistd.h> (pulled in by the core's Arduino.h)
 * declares POSIX `int link(const char*, const char*)`, which collides with a
 * global-scope variable of that name.  See the longer note in pad_node.cpp. */
static GseLink gseLink;

/* ------------------------------------------------------------------ HAL */
extern "C" {

uint32_t gse_millis(void) { return millis(); }

void gse_tx_write(const uint8_t *buf, size_t len)
{
    /* The portable core prefixes each air packet with the LoRa fixed-point
     * address + channel (see air_send() in gse_tx_core.c).  sendRaw() keeps
     * that prefix for the module and strips it for ESP-NOW, where it is not
     * addressing but noise. */
    gseLink.sendRaw(buf, len);
}

bool gse_tx_read_byte(uint8_t *out)
{
    if (!gseLink.available()) return false;
    int r = gseLink.read();
    if (r < 0) return false;
    *out = (uint8_t)r;
    return true;
}

void gse_host_write(const uint8_t *buf, size_t len)
{
    Serial.write(buf, len);
}

bool gse_host_read_byte(uint8_t *out)
{
    if (!Serial.available()) return false;
    *out = (uint8_t)Serial.read();
    return true;
}

void gse_log(const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    Serial.printf("LOG %s\n", buf);
}

bool gse_radio_provision(uint8_t level, uint8_t channel_hex, uint16_t key,
                         uint16_t my_addr)
{
    /* One implementation, one place to fix it: on LoRa this is the same
     * DxLr41::configure() the pad node runs, including its verify-after-reset
     * step (see dxlr41.h -- AT+RESET does not save MODE/LEVEL/CHANNEL/MAC).
     * On ESP-NOW it reports whether startup and peer registration succeeded. */
    String err = gseLink.provision(level, channel_hex, key, my_addr, GSE_ADDR_PAD);
    radioOk = (err.length() == 0);
    if (!radioOk) gse_log("link: %s", err.c_str());
#if defined(GSE_LINK_LORA)
    else          gse_log("radio: OK level=%u ch=%02x", level, channel_hex);
#else
    else          gse_log("espnow: ch %d -> peer registered", (int)GSE_NOW_CHANNEL);
#endif
    return radioOk;
}

} /* extern "C" */

/* --------------------------------------------------------------- lifecycle */
void setup()
{
    /* laptop port.  On an S3 this is USB-CDC (ARDUINO_USB_CDC_ON_BOOT makes
     * Serial the CDC device); on an ESP-32S it is UART0 behind the bridge
     * chip.  Same call, same 115200, same cable on the other end.           */
    Serial.begin(GSE_HOST_BAUD);
    /* Named before anything can fail: a TX node and a pad built with different
     * GSE_LINK_* values cannot hear each other, and this is the line that says
     * which one this is. */
    gseLink.begin();
    gse_log("link: %s -- %s", gseLink.transport(), gseLink.describe().c_str());
    gse_tx_init();                           /* provisions the link         */
}

static uint32_t s_lastDiag;

void loop()
{
    gseLink.poll();                  /* no-op for both transports */
    gse_tx_tick();

    /* once a second, say whether the radio is producing anything at all --
     * the laptop console then separates "pad is silent" from "laptop never
     * read the port".  gse_tx_stats() is filled by the core.               */
    if (millis() - s_lastDiag >= 1000) {
        s_lastDiag = millis();
        const gse_tx_stats_t *st = gse_tx_stats();
        static uint32_t lastRx;
        uint32_t rx = st->rx_state + st->rx_event;
        if (rx != lastRx || millis() < 12000) {
            gse_log("tx fwd=%lu state=%lu safe=%lu dropped=%lu radio=%s",
                    (unsigned long)st->forwarded_cmd,
                    (unsigned long)st->rx_state, (unsigned long)st->safe_frames,
                    (unsigned long)st->dropped_stale,
                    st->radio_present ? "ok" : "FAULT");
            lastRx = rx;
        }
    }
}
