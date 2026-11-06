/*
 * console.h - blocking serial console for the ST-LINK virtual COM port
 *
 * USART3 on PD8 (TX) / PD9 (RX), 115200 8N1, which is the port the NUCLEO's
 * ST-LINK exposes as /dev/ttyACM0 on Linux, /dev/cu.usbmodem* on macOS and
 * COMx on Windows. No interrupts, no DMA, no ring buffer: polling is plenty
 * for a human typing commands, and it keeps the firmware tiny and obvious.
 */
#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Configure the UART (call after board_clock_init()). */
void console_init(void);

/** Write one character (blocks until the TX register is free). */
void console_putc(char c);

/** Write a NUL-terminated string. */
void console_puts(const char *s);

/**
 * Minimal printf. Supported: %s %c %d %i %u %x %X %p %% and the 'l' length
 * modifier; field width / zero padding for the integer formats (e.g. %-9s,
 * %4u, %08x). No floating point, no dynamic allocation.
 */
void console_printf(const char *fmt, ...);

/** Variant taking a va_list, for building other logging helpers. */
void console_vprintf(const char *fmt, va_list ap);

/**
 * @brief Read one line of input, echoing as it goes.
 *
 * Blocks until Enter (\r or \n) is received. Handles backspace and both
 * CRLF and LF line endings. Returns the number of characters in @p buf
 * (0 for an empty line) or -1 if the line was too long / cancelled with
 * Ctrl-C.
 */
int console_read_line(char *buf, int max_len);

/** Non-blocking single character read: returns -1 if nothing is waiting. */
int console_getc_nowait(void);

#ifdef __cplusplus
}
#endif
