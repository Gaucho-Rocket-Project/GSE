/*
 * gse_hal.h  --  the four platform services the GSE link code needs
 * ---------------------------------------------------------------------------
 * Everything in this package that matters (frame encoding, CRC, parser, the
 * failsafe rules) is already portable C in gse_protocol.h.  This header names
 * the only four things that differ between an ESP32-S3 and an STM32H7:
 *
 *      gse_millis()              milliseconds since boot, 32-bit, wrapping
 *      gse_tx_write(b,n)         push n bytes to the DX-LR41 UART
 *      gse_tx_read_byte(&b)      true if one byte was waiting from the module
 *      gse_host_write(b,n)       push n bytes to the laptop
 *      gse_host_read_byte(&b)    true if one byte was waiting from the laptop
 *      gse_radio_provision(...)  write + verify the module's AT configuration
 *      gse_log(...)              one text line to the debug console
 *
 * Plus a begin() so each platform can set up its clocks and pins.
 *
 * Provided implementations:
 *      firmware/src/tx/tx_platform_esp32.cpp   (Arduino / ESP32-S3)
 *      firmware/stm32/tx_platform_stm32.c      (STM32Cube HAL / Nucleo-H753ZI)
 */
#ifndef GSE_HAL_H
#define GSE_HAL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t gse_millis(void);
void     gse_tx_write(const uint8_t *buf, size_t len);
bool     gse_tx_read_byte(uint8_t *out);
void     gse_host_write(const uint8_t *buf, size_t len);
bool     gse_host_read_byte(uint8_t *out);
void     gse_log(const char *fmt, ...);

/* Writes MODE1 / LEVEL / CHANNEL / KEY / MAC / PACKET1 / POWE13 / SLEEP2 to
 * the module and verifies every reply.  Returns false on any mismatch; the
 * node keeps running and keeps the pad in its safe state. */
bool     gse_radio_provision(uint8_t level, uint8_t channel_hex,
                             uint16_t key, uint16_t my_addr);

#ifdef __cplusplus
}
#endif
#endif /* GSE_HAL_H */
