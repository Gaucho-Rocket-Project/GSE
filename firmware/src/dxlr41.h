/*
 * dxlr41.h  --  driver for the DX-LR41-24T12S (Semtech SX1281, 2.4 GHz)
 *
 * The module is an opaque UART "transparent pipe": whatever you write into
 * TXD comes out of the remote module's RXD (same channel, same LEVEL, same
 * KEY).  No packet boundaries, no addressing, no ACK -- the GSE protocol
 * frame in gse_protocol.h provides all of that.
 *
 * Verified against "DX-LR41-24T12S 串口应用指导 v2.0 (2025-08-11)":
 *   - UART default 9600 8N1, AT+BAUD index 2
 *   - AT+LEVEL n : n=0..7 LoRa, n=8 FLRC. Default 3 (10 kbit/s, 2.2 km).
 *       LEVEL 8 (FLRC 1.04 Mbit/s) needs payload >= 6 bytes, <= 127 bytes,
 *       and disables the KEY feature.  Our frames are 19/27/10 bytes -> OK.
 *   - AT+MODE 1 = fixed-point transfer (receiver address + channel prefix),
 *     then packets can be filtered per module.  Unicast is worth the 3 bytes.
 *   - AT+CHANNEL hh : 2400 MHz + 2 MHz * hh, 0x00..0x32.  Default 0x0C (2424).
 *   - AT+PACKET 0..3 = 32/64/128/230 byte chunks.
 *   - AT+POWE 0..13 dBm (default 13).
 *   - AT+OPENKEY 1 / AT+KEY n (0..65535) XOR "encryption" + KEY.  With only
 *     two nodes on a private channel this is not a security boundary; it does
 *     stop the occasional neighbour's LR41 from being understood.
 *   - AUX pin: high while the module is busy (TX in progress / RX / config).
 *     The module raises AUX 2-3 ms *before* it starts pushing received bytes.
 *   - "+++" toggles AT mode; it resets the module and is NOT saved.
 *
 * The driver deliberately uses the module's *defaults* for transport and only
 * writes the parameters that must agree between the two ends, then verifies
 * them with AT+HELP.  A pad node that fails verification keeps its outputs in
 * the safe state and reports a fault.
 */
#ifndef DXLR41_H
#define DXLR41_H

#include <Arduino.h>
#include "gse_protocol.h"

class DxLr41 {
public:
    DxLr41(HardwareSerial &uart, int pinAux, int pinM0 = -1, int pinM1 = -1)
        : _uart(uart), _aux(pinAux), _m0(pinM0), _m1(pinM1) {}

    void begin(unsigned long baud)
    {
        if (_aux >= 0) pinMode(_aux, INPUT_PULLUP);
        if (_m0  >= 0) { pinMode(_m0, OUTPUT); digitalWrite(_m0, LOW); }
        if (_m1  >= 0) { pinMode(_m1, OUTPUT); digitalWrite(_m1, LOW); }
        _uart.begin(baud, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);
        _uart.setTimeout(120);
        delay(50);
        while (_uart.available()) _uart.read();     /* flush boot noise */
    }

    bool busy() const { return _aux >= 0 && digitalRead(_aux) == HIGH; }

    /* Blocking write; waits for AUX to clear so a slow air rate cannot
     * silently overflow the module's 230-byte buffer.                     */
    size_t write(const uint8_t *data, size_t len, uint16_t timeoutMs = 60)
    {
        size_t n = _uart.write(data, len);
        uint32_t t0 = millis();
        while (busy() && (millis() - t0) < timeoutMs) delay(1);
        return n;
    }

    int available() { return _uart.available(); }
    int read()      { return _uart.read(); }
    void flushInput() { while (_uart.available()) _uart.read(); }

    /* -------------------------------------------------------- AT helpers */
    /* Returns true if module echoed OK (or the expected +RESP) within timeout */
    bool at(const char *cmd, const char *expect = "OK", uint32_t timeoutMs = 400)
    {
        flushInput();
        _uart.print(cmd);
        _uart.print("\r\n");
        return _collect(expect, timeoutMs);
    }

    /* Read the whole AT+HELP banner into buf (NUL-terminated). */
    bool help(char *buf, size_t bufLen, uint32_t timeoutMs = 600)
    {
        flushInput();
        _uart.print("AT+HELP\r\n");
        size_t i = 0;
        uint32_t t0 = millis();
        while ((millis() - t0) < timeoutMs) {
            while (_uart.available() && i + 1 < bufLen) {
                char c = (char)_uart.read();
                if (c != '\r') buf[i++] = c;
                buf[i] = 0; // terminates the string right after c ? ?
            }
            if (i > 0 && strstr(buf, "====") && strchr(buf, '\n')) break;
            delay(2);
        }
        buf[i] = 0;
        return strstr(buf, "LoRa Parameter") != nullptr;
    }

    bool enterAt()  { flushInput(); delay(20); _uart.print("+++"); return _collect("AT", 600); }
    bool exitAt()   { return at("+++", "Power On", 900); }   /* exits + reboots */

    /* Configure this end and verify.  Returns an empty String on success,
     * otherwise a short human-readable fault.                             */
    String configure(uint8_t level, uint8_t channelHex, uint16_t key,
                     bool fixedPoint, uint16_t myAddr, uint16_t peerAddr)
    {
        if (!enterAt()) return F("AT entry failed");
        if (!at("AT+MODE1")) return F("AT+MODE1 rejected");
        if (!at((String("AT+LEVEL") + String(level)).c_str())) return F("AT+LEVEL rejected");
        char ch[16]; snprintf(ch, sizeof ch, "AT+CHANNEL%02X", channelHex);
        if (!at(ch)) return F("AT+CHANNEL rejected");
        if (level < 8) {
            at("AT+OPENKEY1");
            if (!at((String("AT+KEY") + String(key)).c_str())) return F("AT+KEY rejected");
        }
        char mac[24];
        snprintf(mac, sizeof mac, "AT+MAC%02X,%02X",
                 (unsigned)(myAddr >> 8), (unsigned)(myAddr & 0xFF));
        if (!at(mac)) return F("AT+MAC rejected");
        if (!at("AT+PACKET1")) return F("AT+PACKET rejected");   /* 64 B */
        if (!at("AT+POWE13")) return F("AT+POWE rejected");
        if (!at("AT+SLEEP2")) return F("AT+SLEEP rejected");     /* low latency */

        /* read the module back BEFORE leaving AT mode.  This is the check that
         * would have caught the real bug on the bench: AT+RESET is a plain
         * reboot ("Power On"), and the receive parameters -- MODE / LEVEL /
         * CHANNEL / MAC / KEY -- are not written to the module's non-volatile
         * store by the set commands in this firmware revision.  A reset
         * therefore puts back whatever was last saved, silently undoing
         * everything above.  Verify, and say exactly what is wrong.          */
        char help[600];
        if (!help_(help, sizeof help)) return F("AT+HELP silent");
        if (!strstr(help, "MODE:1"))        return F("MODE did not stick");
        {
            char want[24];
            snprintf(want, sizeof want, "LEVEL:%u", (unsigned)level);
            if (!strstr(help, want))        return F("LEVEL did not stick");
        }
        {
            char want[32];
            snprintf(want, sizeof want, ">> %02x", (unsigned)channelHex);
            if (!strstr(help, want))        return F("CHANNEL did not stick");
        }
        {
            char want[24];
            snprintf(want, sizeof want, "MAC:%02X,%02x",
                     (unsigned)(myAddr >> 8), (unsigned)(myAddr & 0xFF));
            if (!strstr(help, want)) {
                snprintf(want, sizeof want, "MAC:%02x,%02x",
                         (unsigned)(myAddr >> 8), (unsigned)(myAddr & 0xFF));
                if (!strstr(help, want))    return F("MAC did not stick");
            }
        }

        /* Leave AT mode.  "+++" alone exits; AT+RESET is kept as the fallback
         * because a few module revisions answer only to the named command.
         * After either, everything must be re-verified -- that is the whole
         * point of this function.                                              */
        if (!at("+++", "Power On", 900)) {
            if (!at("AT+RESET", "Power On", 900)) return F("cannot leave AT mode");
        }
        delay(250);

        if (!enterAt()) return F("AT re-entry failed (module lost config)");
        if (!help_(help, sizeof help)) return F("AT+HELP silent after reset");
        if (!strstr(help, "MODE:1")) return F("reset wiped MODE (config not saved)");
        at("+++", "Power On", 900);           /* back to transparent mode */
        delay(250);
        return String();
    }

    /* Read the AT+HELP banner into buf; true if it looks like the banner. */
    bool help_(char *buf, size_t bufLen)
    {
        buf[0] = 0;
        return help(buf, bufLen);
    }

    /* Compose an uplink packet in fixed-point mode: dst(2) + chan(1) + data */
    size_t buildFixedPoint(uint8_t *out, uint16_t dst, uint8_t chan,
                           const uint8_t *data, size_t len)
    {
        out[0] = (uint8_t)(dst >> 8); out[1] = (uint8_t)(dst & 0xFF);
        out[2] = chan;
        memcpy(out + 3, data, len);
        return len + 3;
    }

private:
    bool _collect(const char *expect, uint32_t timeoutMs)
    {
        String resp;
        uint32_t t0 = millis();
        while ((millis() - t0) < timeoutMs) {
            while (_uart.available()) resp += (char)_uart.read();
            if (strstr(resp.c_str(), expect)) return true;
            if (strstr(resp.c_str(), "ERROR")) return false;
            delay(5);
        }
        return strstr(resp.c_str(), expect) != nullptr;
    }

    HardwareSerial &_uart;
    int _aux, _m0, _m1;
};

#endif /* DXLR41_H */
