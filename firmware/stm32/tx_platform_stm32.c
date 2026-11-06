/*
 * tx_platform_stm32.c  --  STM32Cube HAL platform layer for the TX core
 * -------------------------------------------------------------------------
 * Target: NUCLEO-H753ZI (MB1364E), STM32H753ZIT6, STLINK-V3EC
 *
 *   USART3  PD8/PD9   115200  -> ST-LINK VCP -> laptop        (host side)
 *   USART2  PD5/PD6     9600  -> DX-LR41-24T12S RXD/TXD       (radio side)
 *   USART1  PA9/PA10  115200  -> optional debug console
 *   PD7                       <- DX-LR41 AUX (busy), input pull-up
 *
 * Integration
 * -----------
 * Generate the project with STM32CubeMX / CubeIDE for NUCLEO-H753ZI and enable
 *    USART3  Asynchronous, 115200 8N1   (the ST-LINK virtual COM port)
 *    USART2  Asynchronous,   9600 8N1   (PD5 TX, PD6 RX, AF7)
 *    USART1  Asynchronous, 115200 8N1   (optional debug)
 *    PD7     GPIO_Input, pull-up
 *    SysTick 1 kHz
 * then add this file and gse_tx_core.c to the build, hand the three handles to
 * gse_stm32_bind() from main(), and call gse_stm32_start().
 *
 * If you use PlatformIO instead (see firmware/platformio.ini, env tx-nucleo)
 * the handle pointers are filled automatically from the CubeMX-generated
 * main.c - in that case call gse_stm32_autobind() once at start-up.
 *
 * Everything here is deliberately polling-based: no interrupts are required,
 * which keeps the port easy to audit and impossible to deadlock.
 */
#include "main.h"              /* CubeMX: HAL handles, MX_*_Init prototypes   */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "gse_tx_core.h"

/* ---------------------------------------------------------- handles / pins */
static UART_HandleTypeDef *s_host_uart;     /* USART3 -> ST-LINK VCP        */
static UART_HandleTypeDef *s_radio_uart;    /* USART2 -> DX-LR41            */
static UART_HandleTypeDef *s_debug_uart;    /* USART1 -> optional console    */
static GPIO_TypeDef       *s_aux_port;
static uint16_t            s_aux_pin;
static uint32_t            s_t0;

/* On-board user LEDs, UM2407 §7.6.1: LD1 green = PB0, LD2 yellow = PE1,
 * LD3 red = PB14, all active HIGH.  (Arduino's LED_BLUE macro is wrong on this
 * board -- it names PB7, which is USART1_RX.  Do not use those macros here.)
 *
 *   LD1  one flash per packet SENT
 *   LD2  one flash per packet RECEIVED
 *   LD3  steady on RADIO FAULT
 */
static const uint32_t LED_FLASH_MS = 10u;

void gse_stm32_bind(UART_HandleTypeDef *host, UART_HandleTypeDef *radio,
                    UART_HandleTypeDef *dbg, GPIO_TypeDef *aux_port,
                    uint16_t aux_pin)
{
    s_host_uart  = host;
    s_radio_uart = radio;
    s_debug_uart = dbg;
    s_aux_port   = aux_port;
    s_aux_pin    = aux_pin;
    s_t0 = HAL_GetTick();
}

/* Weak default: the CubeMX symbols most projects generate.  Override by
 * calling gse_stm32_bind() with your own handles if your project differs. */
__attribute__((weak)) void gse_stm32_autobind(void)
{
    extern UART_HandleTypeDef huart3;   /* host  (VCP)    */
    extern UART_HandleTypeDef huart2;   /* radio          */
#ifdef USE_USART1_DEBUG
    extern UART_HandleTypeDef huart1;   /* debug          */
    gse_stm32_bind(&huart3, &huart2, &huart1, AUX_GPIO_Port, AUX_Pin);
#else
    gse_stm32_bind(&huart3, &huart2, NULL, AUX_GPIO_Port, AUX_Pin);
#endif
}

/* ------------------------------------------------------------------ HAL API */
uint32_t gse_millis(void)
{
    return HAL_GetTick() - s_t0;
}

void gse_tx_write(const uint8_t *buf, size_t len)
{
    if (!s_radio_uart) return;
    HAL_UART_Transmit(s_radio_uart, (uint8_t *)buf, (uint16_t)len, 100);
    /* The module raises AUX while it is on air; wait for it to clear so a slow
     * air rate cannot silently overflow the module's 64-byte chunk buffer.  */
    if (s_aux_port) {
        uint32_t t = HAL_GetTick();
        while (HAL_GPIO_ReadPin(s_aux_port, s_aux_pin) == GPIO_PIN_SET &&
               (HAL_GetTick() - t) < 60u) {
            /* spin */
        }
    }
}

bool gse_tx_read_byte(uint8_t *out)
{
    if (!s_radio_uart) return false;
    return HAL_UART_Receive(s_radio_uart, out, 1, 0) == HAL_OK;
}

void gse_host_write(const uint8_t *buf, size_t len)
{
    if (!s_host_uart) return;
    HAL_UART_Transmit(s_host_uart, (uint8_t *)buf, (uint16_t)len, 100);
}

bool gse_host_read_byte(uint8_t *out)
{
    if (!s_host_uart) return false;
    return HAL_UART_Receive(s_host_uart, out, 1, 0) == HAL_OK;
}

void gse_log(const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof buf - 1) n = (int)sizeof buf - 1;
    static const char prefix[] = "LOG ";
    if (s_debug_uart) {
        HAL_UART_Transmit(s_debug_uart, (uint8_t *)prefix, 4, 50);
        HAL_UART_Transmit(s_debug_uart, (uint8_t *)buf, (uint16_t)n, 100);
        HAL_UART_Transmit(s_debug_uart, (uint8_t *)"\r\n", 2, 50);
    }
    /* the laptop console also shows the log, so mirror it on the VCP */
    static const char pfx2[] = "\r\nLOG ";
    static const char nl[] = "\r\n";
    HAL_UART_Transmit(s_host_uart, (uint8_t *)pfx2, 6, 50);
    HAL_UART_Transmit(s_host_uart, (uint8_t *)buf, (uint16_t)n, 100);
    HAL_UART_Transmit(s_host_uart, (uint8_t *)nl, 2, 50);
}

/* ------------------------------------------------------- radio provisioning */
static bool at_cmd(UART_HandleTypeDef *u, const char *cmd, const char *expect,
                   uint32_t timeout_ms)
{
    char resp[96];
    size_t i = 0;
    uint8_t c;
    HAL_UART_Transmit(u, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100);
    HAL_UART_Transmit(u, (uint8_t *)"\r\n", 2, 20);
    uint32_t t = HAL_GetTick();
    resp[0] = 0;
    while ((HAL_GetTick() - t) < timeout_ms) {
        if (HAL_UART_Receive(u, &c, 1, 2) == HAL_OK) {
            if (c != '\r' && i + 1 < sizeof resp) resp[i++] = (char)c;
            resp[i] = 0;
            if (strstr(resp, expect)) return true;
            if (strstr(resp, "ERROR")) return false;
            t = HAL_GetTick();          /* bytes are flowing, extend timeout */
        }
    }
    return strstr(resp, expect) != NULL;
}

bool gse_radio_provision(uint8_t level, uint8_t channel_hex, uint16_t key,
                         uint16_t my_addr)
{
    if (!s_radio_uart) return false;
    char cmd[40];

    if (!at_cmd(s_radio_uart, "+++", "Entry AT", 600)) return false;
    if (!at_cmd(s_radio_uart, "AT+MODE1", "OK", 400)) return false;
    snprintf(cmd, sizeof cmd, "AT+LEVEL%u", (unsigned)level);
    if (!at_cmd(s_radio_uart, cmd, "OK", 400)) return false;
    snprintf(cmd, sizeof cmd, "AT+CHANNEL%02X", (unsigned)channel_hex);
    if (!at_cmd(s_radio_uart, cmd, "OK", 400)) return false;
    if (level < 8) {
        at_cmd(s_radio_uart, "AT+OPENKEY1", "OK", 400);
        snprintf(cmd, sizeof cmd, "AT+KEY%u", (unsigned)key);
        if (!at_cmd(s_radio_uart, cmd, "OK", 400)) return false;
    }
    snprintf(cmd, sizeof cmd, "AT+MAC%02X,%02X",
             (unsigned)(my_addr >> 8), (unsigned)(my_addr & 0xFF));
    if (!at_cmd(s_radio_uart, cmd, "OK", 400)) return false;
    if (!at_cmd(s_radio_uart, "AT+PACKET1", "OK", 400)) return false;
    if (!at_cmd(s_radio_uart, "AT+POWE13", "OK", 400)) return false;
    if (!at_cmd(s_radio_uart, "AT+SLEEP2", "OK", 400)) return false;
    if (!at_cmd(s_radio_uart, "AT+RESET", "Power On", 900)) return false;
    HAL_Delay(150);
    return true;
}

/* --------------------------------------------------------------- lifecycle */
/* Published in tx_platform_stm32.h; repeated here so the file also builds
 * when dropped into a CubeIDE project without it. */
void gse_stm32_bind(UART_HandleTypeDef *host, UART_HandleTypeDef *radio,
                    UART_HandleTypeDef *dbg, GPIO_TypeDef *aux_port,
                    uint16_t aux_pin);
void gse_stm32_start(void);
void gse_stm32_loop(void);

static void leds_init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = GPIO_PIN_0;   HAL_GPIO_Init(GPIOB, &g);   /* LD1 green  */
    g.Pin   = GPIO_PIN_1;   HAL_GPIO_Init(GPIOE, &g);   /* LD2 yellow */
    g.Pin   = GPIO_PIN_14;  HAL_GPIO_Init(GPIOB, &g);   /* LD3 red    */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
    /* boot sweep, so a dead LED is obvious before you read anything into it */
    for (int i = 0; i < 2; ++i) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(80);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(80);
    }
}

static void leds_tick(void)
{
    static uint32_t seen_tx, seen_rx, tx_until, rx_until;
    const gse_tx_stats_t *st = gse_tx_stats();
    uint32_t now = HAL_GetTick();

    if (st->sent_frames != seen_tx) { seen_tx = st->sent_frames;
                                      tx_until = now + LED_FLASH_MS; }
    if (st->rx_frames   != seen_rx) { seen_rx = st->rx_frames;
                                      rx_until = now + LED_FLASH_MS; }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  now < tx_until ? GPIO_PIN_SET
                                                         : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  now < rx_until ? GPIO_PIN_SET
                                                         : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, st->radio_present ? GPIO_PIN_RESET
                                                            : GPIO_PIN_SET);
}

void gse_stm32_start(void)
{
    leds_init();
    gse_stm32_autobind();       /* no-op if gse_stm32_bind() was called first */
    gse_tx_init();
}

void gse_stm32_loop(void)
{
    /* 1 kHz SysTick; the core needs no more than a few hundred Hz */
    gse_tx_tick();
    leds_tick();
}
