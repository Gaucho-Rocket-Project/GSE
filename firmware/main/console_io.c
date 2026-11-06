/*
 * console_io.c - line-oriented console input/output for the ESP32-S3
 *
 * Works with either console backend:
 *   - USB Serial/JTAG  (the "USB" port on the dev board)   <- default
 *   - UART0            (the "COM"/UART port, 115200 8N1)
 *
 * Output uses printf() (buffered output disabled) so log lines and menu text
 * can be mixed freely. Input is read from the driver layer so we can apply a
 * read timeout without fighting the VFS.
 */

#include "console_io.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "sdkconfig.h"

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#elif CONFIG_ESP_CONSOLE_UART
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#endif

static const char *TAG = "console";

#define RX_CHUNK 64
#define READ_TIMEOUT_MS 20

void console_io_init(void)
{
    /* Unbuffered stdout so every line shows up on the host immediately. */
    setvbuf(stdout, NULL, _IONBF, 0);

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        cfg.rx_buffer_size = 1024;
        cfg.tx_buffer_size = 1024;
        esp_err_t err = usb_serial_jtag_driver_install(&cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "USB-Serial-JTAG driver install failed: %s", esp_err_to_name(err));
            return;
        }
    }
    ESP_LOGI(TAG, "Console on USB Serial/JTAG (virtual COM port, any baud rate)");
#elif CONFIG_ESP_CONSOLE_UART
    const uart_port_t port = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;
    esp_err_t err = uart_driver_install(port, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Console: UART%d @ 115200 8N1", (int)CONFIG_ESP_CONSOLE_UART_NUM);
#else
    ESP_LOGW(TAG, "No serial console configured");
#endif
}

int console_io_read_byte(void)
{
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    uint8_t c;
    if (usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(READ_TIMEOUT_MS)) == 1) {
        return (int)c;
    }
    return -1;
#elif CONFIG_ESP_CONSOLE_UART
    uint8_t c;
    if (uart_read_bytes((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM, &c, 1,
                        pdMS_TO_TICKS(READ_TIMEOUT_MS)) == 1) {
        return (int)c;
    }
    return -1;
#else
    return -1;
#endif
}

int console_io_read_line(char *buf, size_t maxlen)
{
    size_t len = 0;

    for (;;) {
        int c = console_io_read_byte();
        if (c < 0) {
            continue;   /* nothing typed yet */
        }

        if (c == '\r' || c == '\n') {
            /* Swallow the LF of a CRLF pair by peeking once; harmless if absent. */
            console_io_write_char('\n');
            buf[len] = '\0';
            return (int)len;
        }

        if (c == 0x7F || c == 0x08) {       /* backspace / delete */
            if (len > 0) {
                len--;
                console_io_write_str("\b \b");
            }
            continue;
        }

        if (c < 0x20 || c > 0x7E) {
            continue;   /* ignore control / non-ASCII bytes */
        }

        if (len + 1 < maxlen) {
            buf[len++] = (char)c;
            console_io_write_char((char)c);   /* local echo */
        }
    }
}

void console_io_write_str(const char *s)
{
    fputs(s, stdout);
    fflush(stdout);
}

void console_io_write_char(char c)
{
    fputc(c, stdout);
    fflush(stdout);
}
