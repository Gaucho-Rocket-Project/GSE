/*
 * tx_platform_nucleo.cpp  --  Arduino-core platform layer for the Nucleo-H753ZI
 * ---------------------------------------------------------------------------
 * Board : NUCLEO-H753ZI (MB1364E, STM32H753ZIT6, STLINK-V3EC)
 * Core  : arduino (stm32duino), env tx-nucleo in platformio.ini
 *
 * Ports
 *   Serial  = USART3  PD8/PD9  -> ST-LINK virtual COM port -> laptop, 115200
 *   Serial1 = USART2  PD5/PD6  -> DX-LR41-24T12S RXD/TXD,     9600
 *   Serial2 = USART1  PA9/PA10 -> optional debug console,    115200
 *   PD7                        <- DX-LR41 AUX
 *
 * If you would rather use STM32Cube HAL / CubeIDE, use
 * stm32/tx_platform_stm32.c instead; both feed the same portable core in
 * src/common/gse_tx_core.c, so the laptop and the pad cannot tell them apart.
 */
#if defined(GSE_PLATFORM_STM32)

#include <Arduino.h>
#include <stdarg.h>

#include "config.h"
#include "gse_hal.h"
#include "gse_tx_core.h"

/* Pin map - change here, nowhere else ---------------------------------------- */
static const uint32_t PIN_RADIO_AUX = PD7;  /* CN11-45, module busy          */

/* On-board user LEDs (UM2407 §7.6.1).  Do NOT use Arduino's LED_GREEN /
 * LED_BLUE / LED_RED macros here: this board's variant header defines
 * LED_BLUE as PB7, and PB7 is USART1_RX / LPUART1_RX on the H753 Nucleo --
 * driving it would fight the debug UART.  UM2407 is explicit: LD1 green = PB0,
 * LD2 yellow = PE1, LD3 red = PB14, all active HIGH.
 *
 *   LD1 green   one short flash per packet SENT to the air
 *   LD2 yellow  one short flash per packet RECEIVED from the air
 *   LD3 red     steady ON while RADIO FAULT (module did not provision)
 *
 * At 20 Hz a sent-packet flash is 10 ms on / 40 ms off, which reads as a fast
 * flicker when the link is healthy and as "nothing at all" when it is not --
 * exactly the question you are asking the board.
 */
static const uint32_t PIN_LED_TX = PB0;    /* LD1 green  */
static const uint32_t PIN_LED_RX = PE1;    /* LD2 yellow */
static const uint32_t PIN_LED_ERR = PB14;  /* LD3 red    */

#define LED_FLASH_MS 10u

static uint32_t s_led_tx_until, s_led_rx_until;
static uint32_t s_seen_tx, s_seen_rx;

static inline void led_set(uint32_t pin, bool on)
{
    digitalWrite(pin, on ? HIGH : LOW);
}

/* Called from loop() at whatever rate the main loop runs.  It compares the
 * core's counters rather than instrumenting gse_tx_write(), so it keeps
 * working if the platform layer is swapped out. */
static void leds_tick(void)
{
    const gse_tx_stats_t *st = gse_tx_stats();
    uint32_t now = millis();

    if (st->sent_frames != s_seen_tx) {
        s_seen_tx  = st->sent_frames;
        s_led_tx_until = now + LED_FLASH_MS;
    }
    if (st->rx_frames != s_seen_rx) {
        s_seen_rx  = st->rx_frames;
        s_led_rx_until = now + LED_FLASH_MS;
    }
    led_set(PIN_LED_TX,  now < s_led_tx_until);
    led_set(PIN_LED_RX,  now < s_led_rx_until);
    led_set(PIN_LED_ERR, !st->radio_present);
}

static bool s_radio_ok = false;

/* ------------------------------------------------------------------- HAL */
extern "C" {

uint32_t gse_millis(void) { return millis(); }

void gse_tx_write(const uint8_t *buf, size_t len)
{
    Serial1.write(buf, len);
    Serial1.flush();
    /* AUX goes high while the module is on air; waiting for it to fall keeps a
     * 20 Hz stream from overrunning the module's buffer at low air rates.   */
    uint32_t t0 = millis();
    while (digitalRead(PIN_RADIO_AUX) == HIGH && (millis() - t0) < 60u) {
        /* spin: worst case 60 ms, well inside the 50 ms command period budget
           because the module clears AUX as soon as the packet is out.       */
    }
}

bool gse_tx_read_byte(uint8_t *out)
{
    if (Serial1.available() == 0) return false;
    *out = (uint8_t)Serial1.read();
    return true;
}

void gse_host_write(const uint8_t *buf, size_t len)
{
    Serial.write(buf, len);
    Serial.flush();
}

bool gse_host_read_byte(uint8_t *out)
{
    if (Serial.available() == 0) return false;
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
    Serial1.printf("LOG %s\r\n", buf);     /* debug header  */
    Serial.printf("\r\nLOG %s\r\n", buf);  /* and the laptop console */
}

/* ------------------------------------------------------- AT provisioning */
static bool at_cmd(const char *cmd, const char *expect, uint32_t timeout_ms)
{
    char resp[96];
    size_t i = 0;
    while (Serial1.available()) Serial1.read();          /* flush            */
    Serial1.print(cmd);
    Serial1.print("\r\n");
    uint32_t t = millis();
    resp[0] = 0;
    while ((millis() - t) < timeout_ms) {
        if (Serial1.available()) {
            char c = (char)Serial1.read();
            if (c != '\r' && i + 1 < sizeof resp) resp[i++] = c;
            resp[i] = 0;
            if (strstr(resp, expect)) return true;
            if (strstr(resp, "ERROR")) return false;
            t = millis();
        }
    }
    return strstr(resp, expect) != NULL;
}

/* Pull the whole AT+HELP banner. */
static bool at_help(char *buf, size_t bufLen)
{
    size_t i = 0;
    while (Serial1.available()) Serial1.read();
    Serial1.print("AT+HELP\r\n");
    uint32_t t = millis();
    while ((millis() - t) < 1200 && i + 1 < bufLen) {
        if (Serial1.available()) {
            char c = (char)Serial1.read();
            if (c != '\r') buf[i++] = c;
            t = millis();
        }
    }
    buf[i] = 0;
    return strstr(buf, "LoRa Parameter") != NULL;
}

bool gse_radio_provision(uint8_t level, uint8_t channel_hex, uint16_t key,
                         uint16_t my_addr)
{
    char cmd[40];
    if (!at_cmd("+++", "Entry AT", 700)) return false;
    if (!at_cmd("AT+MODE1", "OK", 400)) return false;
    snprintf(cmd, sizeof cmd, "AT+LEVEL%u", (unsigned)level);
    if (!at_cmd(cmd, "OK", 400)) return false;
    snprintf(cmd, sizeof cmd, "AT+CHANNEL%02X", (unsigned)channel_hex);
    if (!at_cmd(cmd, "OK", 400)) return false;
    if (level < 8) {
        at_cmd("AT+OPENKEY1", "OK", 400);
        snprintf(cmd, sizeof cmd, "AT+KEY%u", (unsigned)key);
        if (!at_cmd(cmd, "OK", 400)) return false;
    }
    snprintf(cmd, sizeof cmd, "AT+MAC%02X,%02X",
             (unsigned)(my_addr >> 8), (unsigned)(my_addr & 0xFF));
    if (!at_cmd(cmd, "OK", 400)) return false;
    if (!at_cmd("AT+PACKET1", "OK", 400)) return false;
    if (!at_cmd("AT+POWE13", "OK", 400)) return false;
    if (!at_cmd("AT+SLEEP2", "OK", 400)) return false;

    /* verify before leaving AT mode: AT+RESET is a reboot, not a save, and the
     * receive parameters are not in NVM, so a reset silently reverts them.  */
    char help[600];
    if (!at_help(help, sizeof help)) return false;
    if (!strstr(help, "MODE:1")) return false;
    {
        char want[24];
        snprintf(want, sizeof want, "LEVEL:%u", (unsigned)level);
        if (!strstr(help, want)) return false;
        snprintf(want, sizeof want, ">> %02x", (unsigned)channel_hex);
        if (!strstr(help, want)) return false;
    }

    if (!at_cmd("+++", "Power On", 900) && !at_cmd("AT+RESET", "Power On", 900))
        return false;
    delay(250);

    /* and verify again after the reboot -- this is the step that catches a
     * module which does not keep its configuration across a reset.          */
    if (!at_cmd("+++", "Entry AT", 700)) return false;
    bool ok = at_help(help, sizeof help) && strstr(help, "MODE:1");
    at_cmd("+++", "Power On", 900);           /* back to transparent mode */
    delay(250);
    return ok;
}

} /* extern "C" */

/* --------------------------------------------------------------- lifecycle */
void setup()
{
    pinMode(PIN_RADIO_AUX, INPUT_PULLUP);

    pinMode(PIN_LED_TX, OUTPUT);
    pinMode(PIN_LED_RX, OUTPUT);
    pinMode(PIN_LED_ERR, OUTPUT);
    /* quick three-LED sweep at boot: proves all three work before you start
     * reading anything into the blink patterns                              */
    for (int i = 0; i < 2; ++i) {
        digitalWrite(PIN_LED_TX,  HIGH); digitalWrite(PIN_LED_RX,  HIGH);
        digitalWrite(PIN_LED_ERR, HIGH); delay(80);
        digitalWrite(PIN_LED_TX,  LOW);  digitalWrite(PIN_LED_RX,  LOW);
        digitalWrite(PIN_LED_ERR, LOW);  delay(80);
    }

    /* stm32duino names its Uart objects after the peripheral, not after an
     * index: Serial3 = USART3 and Serial is an alias for it (the board sets
     * SERIAL_UART_INSTANCE 3). build_flags adds ENABLE_HWSERIAL1/2 so that
     * Serial1 = USART1 and Serial2 = USART2 also exist.                    */
    Serial.begin(115200);                       /* USART3 VCP -> laptop 115200 */

    Serial2.setRx(PD_6);                        /* CN11-43  <- module TXD      */
    Serial2.setTx(PD_5);                        /* CN11-41  -> module RXD      */
    Serial2.begin(9600);                        /* DX-LR41-24T12S              */

    Serial1.setRx(PA_10);                       /* optional debug console      */
    Serial1.setTx(PA_9);
    Serial1.begin(115200);

    s_radio_ok = false;
    gse_tx_init();                       /* prints ready / RADIO FAULT       */
}

void loop()
{
    gse_tx_tick();
    leds_tick();
}

#endif /* GSE_PLATFORM_STM32 */
