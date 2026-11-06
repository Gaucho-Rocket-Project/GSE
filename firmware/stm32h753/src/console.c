/*
 * console.c - polled USART3 driver + a very small printf
 *
 * The NUCLEO-H753ZI routes USART3 (PD8 = TX, PD9 = RX) to the on-board
 * ST-LINK, which turns it into the "STMicroelectronics STLink Virtual COM
 * Port". That is why no USB stack, no CDC driver and no extra cable are
 * needed: one USB cable to the ST-LINK connector gives you flash + console.
 */
#include "console.h"

#include "board.h"

/* ------------------------------------------------------------------------- */
/* Low level UART                                                            */
/* ------------------------------------------------------------------------- */

void console_init(void)
{
    RCC_APB1LENR |= RCC_APB1LENR_USART3EN;
    (void)RCC_APB1LENR;
    RCC_AHB4ENR |= RCC_AHB4ENR_GPIODEN;
    (void)RCC_AHB4ENR;

    /* PD8 -> USART3_TX (AF7), PD9 -> USART3_RX (AF7). */
    board_gpio_af(GPIOD, CONSOLE_TX_PIN, AF7_USART3);
    board_gpio_af(GPIOD, CONSOLE_RX_PIN, AF7_USART3);

    CONSOLE_USART->CR1 = 0;             /* disable while configuring */
    CONSOLE_USART->CR2 = 0;             /* 1 stop bit, 8 data bits      */
    CONSOLE_USART->CR3 = 0;             /* no flow control              */

    /* USART3 is on APB1 (100 MHz). BRR = fCK / baud, with rounding. */
    CONSOLE_USART->BRR = (BOARD_PCLK1_HZ + BOARD_CONSOLE_BAUD / 2UL) / BOARD_CONSOLE_BAUD;

    CONSOLE_USART->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void console_putc(char c)
{
    /* Wait for the transmit register to empty, then hand over the byte. */
    while ((CONSOLE_USART->ISR & USART_ISR_TXE) == 0UL) {
    }
    CONSOLE_USART->TDR = (uint32_t)(uint8_t)c;
}

void console_puts(const char *s)
{
    if (s == 0) {
        return;
    }
    while (*s != '\0') {
        if (*s == '\n') {
            console_putc('\r');         /* CRLF: some terminals need it */
        }
        console_putc(*s++);
    }
}

int console_getc_nowait(void)
{
    if ((CONSOLE_USART->ISR & USART_ISR_RXNE) != 0UL) {
        return (int)(CONSOLE_USART->RDR & 0xFFUL);
    }
    return -1;
}

int console_read_line(char *buf, int max_len)
{
    int  len    = 0;
    bool too_long = false;

    if (buf == 0 || max_len <= 1) {
        return -1;
    }

    for (;;) {
        int c = console_getc_nowait();
        if (c < 0) {
            continue;
        }

        if (c == '\r' || c == '\n') {
            console_puts("\r\n");
            buf[len] = '\0';
            return too_long ? -1 : len;
        }

        if (c == 3) {                   /* Ctrl-C: abandon the line */
            console_puts("^C\r\n");
            buf[0] = '\0';
            return -1;
        }

        if (c == 8 || c == 127) {       /* Backspace / DEL */
            if (len > 0) {
                len--;
                console_puts("\b \b");
            }
            continue;
        }

        if (c < 32 || c > 126) {        /* ignore control / non-ASCII */
            continue;
        }

        if (len >= max_len - 1) {
            too_long = true;            /* keep eating until Enter */
            continue;
        }

        buf[len++] = (char)c;
        console_putc((char)c);          /* echo */
    }
}

/* ------------------------------------------------------------------------- */
/* Minimal printf                                                            */
/* ------------------------------------------------------------------------- */

static void put_unsigned(uint32_t v, unsigned base, bool upper, int width, char pad,
                         bool left)
{
    char     tmp[12];
    unsigned n = 0;

    if (v == 0UL) {
        tmp[n++] = '0';
    }
    while (v != 0UL) {
        unsigned d = (unsigned)(v % base);
        tmp[n++] = (char)(d < 10U ? ('0' + d) : ((upper ? 'A' : 'a') + (d - 10U)));
        v /= base;
    }
    int pad_count = width - (int)n;
    if (!left) {
        while (pad_count-- > 0) {
            console_putc(pad);
        }
    }
    while (n > 0U) {
        console_putc(tmp[--n]);
    }
    if (left) {
        while (pad_count-- > 0) {
            console_putc(' ');
        }
    }
}

static void put_signed(int32_t v, int width, char pad, bool left)
{
    if (v < 0) {
        console_putc('-');
        if (width > 0) {
            width--;
        }
        put_unsigned((uint32_t)(-(v + 1)) + 1UL, 10, false, width, pad, left);
    } else {
        put_unsigned((uint32_t)v, 10, false, width, pad, left);
    }
}

void console_vprintf(const char *fmt, va_list ap)
{
    while (*fmt != '\0') {
        if (*fmt != '%') {
            if (*fmt == '\n') {
                console_putc('\r');
            }
            console_putc(*fmt++);
            continue;
        }

        fmt++;                             /* skip '%' */
        if (*fmt == '\0') {
            break;
        }

        /* flags: '-' left align, '0' zero pad */
        bool zero_pad = false;
        bool left_align = false;
        while (*fmt == '-' || *fmt == '0' || *fmt == ' ' || *fmt == '+') {
            if (*fmt == '0') {
                zero_pad = true;
            } else if (*fmt == '-') {
                left_align = true;
            }
            fmt++;
        }

        int width = 0;
        while (*fmt >= '0' && *fmt <= '9') {
            width = width * 10 + (*fmt - '0');
            fmt++;
        }
        while (*fmt == 'l' || *fmt == 'h' || *fmt == 'z') {
            fmt++;                         /* length modifiers: ignored */
        }

        char pad = (zero_pad && !left_align) ? '0' : ' ';

        switch (*fmt) {
        case 's': {
            const char *s = va_arg(ap, const char *);
            if (s == 0) {
                s = "(null)";
            }
            int n = 0;
            while (s[n] != '\0') {
                n++;
            }
            if (!left_align) {
                for (int i = n; i < width; i++) {
                    console_putc(' ');
                }
            }
            while (*s != '\0') {
                console_putc(*s++);
            }
            if (left_align) {
                for (int i = n; i < width; i++) {
                    console_putc(' ');
                }
            }
            break;
        }
        case 'c':
            console_putc((char)va_arg(ap, int));
            break;
        case 'd':
        case 'i':
            put_signed((int32_t)va_arg(ap, int), width, pad, left_align);
            break;
        case 'u':
            put_unsigned((uint32_t)va_arg(ap, unsigned), 10, false, width, pad, left_align);
            break;
        case 'x':
            put_unsigned((uint32_t)va_arg(ap, unsigned), 16, false, width, pad, left_align);
            break;
        case 'X':
            put_unsigned((uint32_t)va_arg(ap, unsigned), 16, true, width, pad, left_align);
            break;
        case 'p':
            console_puts("0x");
            put_unsigned((uint32_t)(uintptr_t)va_arg(ap, void *), 16, false, 8, '0', false);
            break;
        case '%':
            console_putc('%');
            break;
        default:
            /* Unknown conversion: print it literally so bugs are visible. */
            console_putc('%');
            console_putc(*fmt);
            break;
        }
        fmt++;
    }
}

void console_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    console_vprintf(fmt, ap);
    va_end(ap);
}
