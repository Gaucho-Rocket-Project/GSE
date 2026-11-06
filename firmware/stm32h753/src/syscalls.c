/*
 * syscalls.c - the few hook functions a freestanding C build may need
 *
 * This firmware never uses libc stdio (console.c has its own tiny formatter),
 * so there is no malloc, no FILE and no errno: every hook here is a deliberate
 * stub. They only exist in case the toolchain emits a call to one of them.
 */
#include <stddef.h>
#include <stdint.h>

#include "board.h"

/* Defined by link.ld: first free RAM address. */
extern char end;

void *_sbrk(ptrdiff_t incr)
{
    /* No heap. Returning -1 makes any accidental malloc() fail loudly. */
    (void)incr;
    return (void *)-1;
}

int _write(int fd, const char *buf, int len)
{
    /* Route libc printf (should it ever be pulled in) to the console UART. */
    (void)fd;
    for (int i = 0; i < len; i++) {
        while ((USART3->ISR & USART_ISR_TXE) == 0UL) {
        }
        USART3->TDR = (uint32_t)(uint8_t)buf[i];
    }
    return len;
}

int _read(int fd, char *buf, int len)
{
    (void)fd;
    (void)buf;
    (void)len;
    return 0;
}

int _close(int fd) { (void)fd; return -1; }
int _fstat(int fd, void *st) { (void)fd; (void)st; return -1; }
int _isatty(int fd) { (void)fd; return 1; }
int _lseek(int fd, int off, int whence) { (void)fd; (void)off; (void)whence; return 0; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }
int _getpid(void) { return 1; }

void _exit(int status)
{
    (void)status;
    for (;;) {
    }
}

/* libgloss-style aliases some toolchains look for. */
int write(int fd, const char *buf, int len) { return _write(fd, buf, len); }

/*
 * -------------------------------------------------------------------------
 * memcpy / memset / memmove
 * -------------------------------------------------------------------------
 * A -nostdlib build has no C library, but the compiler is allowed to turn a
 * struct assignment or a loop into a call to one of these. Providing them
 * keeps the link working no matter how aggressive the optimiser gets (and
 * they are used by startup.c for the .data copy).
 */
void *memcpy(void *dst, const void *src, size_t n)
{
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    while (n-- > 0u) {
        *d++ = *s++;
    }
    return dst;
}

void *memmove(void *dst, const void *src, size_t n)
{
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    if (d < s) {
        while (n-- > 0u) {
            *d++ = *s++;
        }
    } else if (d > s) {
        d += n;
        s += n;
        while (n-- > 0u) {
            *--d = *--s;
        }
    }
    return dst;
}

void *memset(void *dst, int c, size_t n)
{
    unsigned char *d = (unsigned char *)dst;
    while (n-- > 0u) {
        *d++ = (unsigned char)c;
    }
    return dst;
}
