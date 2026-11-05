/*
 * console_io.h - serial console helpers (USB Serial/JTAG or UART0)
 */
#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Configure the console backend for this build (see sdkconfig.defaults). */
void console_io_init(void);

/** Blocking read of one line (without CR/LF). Returns line length. */
int console_io_read_line(char *buf, size_t maxlen);

/** Read a single byte, returns -1 if nothing arrived within the read timeout. */
int console_io_read_byte(void);

void console_io_write_str(const char *s);
void console_io_write_char(char c);

#ifdef __cplusplus
}
#endif
