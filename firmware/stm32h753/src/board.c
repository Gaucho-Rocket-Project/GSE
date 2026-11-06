/*
 * board.c - clocks, GPIO, and microsecond time base for the NUCLEO-H753ZI
 *
 * Clock tree produced here (all from the internal 64 MHz RC oscillator, so no
 * external crystal, no ST-LINK MCO dependency, nothing to get wrong on a
 * breadboard):
 *
 *   HSI 64 MHz / DIVM1=32 -> 2 MHz  PLL1 reference   (2..16 MHz range)
 *   VCO = 2 MHz x DIVN=200 = 400 MHz                 (192..836 MHz wide range)
 *   PLL1P = 400 / 2 = 200 MHz = SYSCLK
 *   AHB  = SYSCLK / 1        = 200 MHz  (HCLK, core + GPIO)
 *   APB1 = HCLK / 2          = 100 MHz  (USART3)
 *   APB2 = HCLK / 2          = 100 MHz  (TIM1, TIM8, TIM15..17)
 *   timer kernel clock       = 2 x PCLK = 200 MHz
 *
 * Timing accuracy: the H753 HSI is factory-trimmed to +-1 % over temperature,
 * which is far better than a hobby servo needs (the servo's own dead band is
 * ~3 us, i.e. 0.2 % of the 1500 us centre pulse).
 */
#include "board.h"

/* Reset value of RCC_CFGR's SWS field is 0 (HSI), but after a soft reset the
 * PLL may still be running; set it back to a known state first. */
static void clock_fallback_hsi(void)
{
    /* Run from HSI first... */
    RCC->CFGR &= ~(7UL << RCC_CFGR_SW_Pos);
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != 0UL) {
        /* wait for HSI as system clock */
    }

    /*
     * ...then make sure PLL1 is stopped before its registers are rewritten.
     * This matters after a debugger soft reset (`openocd ... reset`) or when
     * replacing another program: the PLL can still be running with someone
     * else's dividers, and writes to PLL1DIVR/PLLCFGR are ignored while
     * PLL1RDY is set - which would leave the core running at whatever
     * frequency the previous firmware chose.
     */
    RCC->CR &= ~RCC_CR_PLL1ON;
    while ((RCC->CR & RCC_CR_PLL1RDY) != 0UL) {
    }
}

static void power_init(void)
{
    /* 1. PWR clock on. */
    RCC_APB4ENR |= RCC_APB4ENR_PWREN;
    (void)RCC_APB4ENR;

    /* 2. Talk to the PWR block unlocked: SDEN must be 0 for LDO mode. */
    PWR->CR3 = (PWR->CR3 & ~((1UL << 4) | (1UL << 3))) | PWR_CR3_LDOEN;

    /* 3. Wait until the regulator reports a valid voltage (ACTVOSRDY). Plain
     *    RAM writes are not allowed before this flag is set. */
    uint32_t guard = 0;
    while ((PWR->CSR1 & PWR_CSR1_ACTVOSRDY) == 0UL) {
        if (++guard > 0xFFFFFUL) {
            break;      /* don't hang forever if something is very wrong */
        }
    }

    /* 4. Voltage scale 1: 200 MHz needs VOS1 on the H753 (VOS0 exists but is
     *    only needed above 200 MHz). */
    PWR->D3CR = (PWR->D3CR & ~(3UL << PWR_D3CR_VOS_Pos)) | PWR_D3CR_VOS1;
    while ((PWR->D3CR & (1UL << 13)) == 0UL) {
        /* VOSRDY */
    }

    /* 5. The internal LDO is the only supply option that needs no board
     *    modification; the H753 NUCLEO defaults to it (no SMPS fitted). */
}

static void flash_latency_init(void)
{
    /* HCLK 200 MHz at VOS1 with VDD = 3.3 V needs 2 wait states, and the
     * programming delay code for "high frequency" must be 1 (RM0433 table
     * "Number of wait states according to CPU clock"). */
    uint32_t acr = FLASH->ACR;
    acr &= ~FLASH_ACR_LATENCY_Msk;
    acr |= FLASH_ACR_LATENCY_2WS;
    acr |= (1UL << 4);              /* WRHIGHFREQ = 0b01 */
    FLASH->ACR = acr;

    /* Read back to make sure it stuck before we speed the core up. */
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_2WS) {
    }
}

static void pll1_init(void)
{
    /* PLL1 is fed by HSI/1 (64 MHz); make sure that is selected. */
    RCC->PLLCKSELR = (RCC->PLLCKSELR & ~(3UL << 0)) | RCC_PLLCKSELR_PLLSRC_HSI;

    /* DIVM1 = 32 -> 2 MHz reference. */
    RCC->PLLCKSELR = (RCC->PLLCKSELR & ~(0x3FUL << RCC_PLLCKSELR_DIVM1_Pos)) |
                     (32UL << RCC_PLLCKSELR_DIVM1_Pos);

    /* Reference clock range: the PLL input is 2.0 MHz, which belongs to the
     * 2..4 MHz band (PLL1RGE = 0b01). VCOSEL = 0 selects the wide VCO range
     * (192..836 MHz) and FRACEN stays 0 so the VCO runs at an exact integer
     * 400 MHz. P (SYSCLK) and R are the outputs we enable. */
    RCC->PLLCFGR = (1UL << RCC_PLLCFGR_PLL1RGE_Pos)
                 | RCC_PLLCFGR_DIVP1EN
                 | RCC_PLLCFGR_DIVR1EN;

    /* DIVN = 200, DIVP = 2, DIVQ = 2, DIVR = 2. */
    RCC->PLL1DIVR = ((200UL - 1UL) << RCC_PLL1DIVR_PLL1N_Pos) |
                    ((2UL - 1UL) << RCC_PLL1DIVR_PLL1P_Pos) |
                    ((2UL - 1UL) << RCC_PLL1DIVR_PLL1Q_Pos) |
                    ((2UL - 1UL) << RCC_PLL1DIVR_PLL1R_Pos);
    RCC->PLL1FRACR = 0UL;

    /* Start PLL1 and wait for lock. */
    RCC->CR |= RCC_CR_PLL1ON;
    while ((RCC->CR & RCC_CR_PLL1RDY) == 0UL) {
    }
}

void board_early_init(void)
{
    /*
     * Runs *before* .data/.bss are set up, exactly like ST's SystemInit() in
     * the CubeIDE startup file. That ordering matters on the H7: until the
     * voltage regulator is configured and ACTVOSRDY is set, write accesses to
     * RAM are not permitted, so the .data copy must come after this.
     * Nothing here touches a global variable or a libc function.
     */
    power_init();           /* LDO supply, VOS1, wait for ACTVOSRDY */
    flash_latency_init();   /* 2 wait states + WRHIGHFREQ for 200 MHz */
}

void board_clock_init(void)
{
    /* HSI on (it is on out of reset, but be explicit). */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0UL) {
    }
    clock_fallback_hsi();
    pll1_init();

    /* Bus prescalers: AHB /1, APB1 /2, APB2 /2, APB4 /1, CPU /1.
     * Configured BEFORE switching the system clock to PLL1. */
    RCC->D1CFGR = (RCC_PRE_DIV1 << RCC_D1CFGR_HPRE_Pos) |
                  (RCC_PRE_DIV1 << RCC_D1CFGR_D1CPRE_Pos);
    RCC->D2CFGR = (RCC_PRE_DIV2 << RCC_D2CFGR_D2PPRE1_Pos) |
                  (RCC_PRE_DIV2 << RCC_D2CFGR_D2PPRE2_Pos);
    RCC->D3CFGR = (RCC_PRE_DIV1 << RCC_D3CFGR_D3PPRE_Pos);

    /* Switch SYSCLK to PLL1. */
    RCC->CFGR = (RCC->CFGR & ~(7UL << RCC_CFGR_SW_Pos)) | RCC_CFGR_SW_PLL1;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL1) {
    }
}

void board_gpio_af(GPIO_Regs *port, unsigned pin, unsigned af)
{
    /* Alternate function: MODER = 10, high speed, no pull. */
    port->MODER = (port->MODER & ~GPIO_MODER_Msk(pin)) | GPIO_MODER_AF(pin);
    port->OSPEEDR = (port->OSPEEDR & ~GPIO_MODER_Msk(pin)) | GPIO_OSPEEDR_HIGH(pin);
    port->PUPDR = (port->PUPDR & ~GPIO_MODER_Msk(pin)) | GPIO_PUPDR_NONE(pin);
    port->OTYPER &= ~(1UL << pin);          /* push-pull */

    if (pin < 8) {
        port->AFR[0] = (port->AFR[0] & ~(0xFUL << (4 * pin))) | ((uint32_t)af << (4 * pin));
    } else {
        unsigned sh = 4 * (pin - 8);
        port->AFR[1] = (port->AFR[1] & ~(0xFUL << sh)) | ((uint32_t)af << sh);
    }
}

void board_gpio_output(GPIO_Regs *port, unsigned pin)
{
    port->MODER = (port->MODER & ~GPIO_MODER_Msk(pin)) | GPIO_MODER_OUTPUT(pin);
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR = (port->OSPEEDR & ~GPIO_MODER_Msk(pin)) | GPIO_OSPEEDR_HIGH(pin);
    port->PUPDR = (port->PUPDR & ~GPIO_MODER_Msk(pin)) | GPIO_PUPDR_NONE(pin);
}

void board_dwt_init(void)
{
    /* Enable the trace/debug block so DWT_CYCCNT can be programmed. */
    DEMCR |= (1UL << 24);                   /* TRCENA */
    DWT_CYCCNT = 0;
    DWT_CTRL |= (1UL << 0);                 /* CYCCNTENA */
}

uint32_t board_cycles(void)
{
    return DWT_CYCCNT;
}

uint32_t board_millis(void)
{
    /* 200 MHz / 200 000 = 1 ms. 32-bit counter wraps every ~21 s, so the
     * division has to happen on the full cycle count, not on the difference. */
    return DWT_CYCCNT / (BOARD_SYSCLK_HZ / 1000UL);
}

void board_delay_ms(uint32_t ms)
{
    /* Accurately timed off the DWT cycle counter. Unsigned subtraction keeps
     * this correct across the 32-bit counter wrap. */
    const uint32_t ticks_per_ms = BOARD_SYSCLK_HZ / 1000UL;
    while (ms-- > 0UL) {
        uint32_t start = DWT_CYCCNT;
        while ((uint32_t)(DWT_CYCCNT - start) < ticks_per_ms) {
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Layout / register-map sanity checks (compile time)                        */
/* ------------------------------------------------------------------------- */
#if defined(__GNUC__)
_Static_assert(sizeof(TIM_Regs) == 0x48, "TIM register block size");
_Static_assert(sizeof(USART_Regs) == 0x2C, "USART register block size");
_Static_assert(sizeof(GPIO_Regs) == 0x28, "GPIO register block size");
_Static_assert(sizeof(PWR_Regs) == 0x1C, "PWR register block size");
_Static_assert(sizeof(FLASH_Regs) == 0x1C, "FLASH register block size");
_Static_assert(RCC_BASE == 0x58024400UL, "RCC base address");
_Static_assert(PWR_BASE == 0x58024800UL, "PWR base address");
/* Offset check for the RCC_Regs fields that matter, using the struct itself. */
#define RCC_OFF(f) ((uint32_t)(__builtin_offsetof(RCC_Regs, f)))
_Static_assert(RCC_OFF(D1CFGR) == 0x18, "RCC.D1CFGR");
_Static_assert(RCC_OFF(D2CFGR) == 0x1C, "RCC.D2CFGR");
_Static_assert(RCC_OFF(D3CFGR) == 0x20, "RCC.D3CFGR");
_Static_assert(RCC_OFF(PLLCKSELR) == 0x28, "RCC.PLLCKSELR");
_Static_assert(RCC_OFF(PLLCFGR) == 0x2C, "RCC.PLLCFGR");
_Static_assert(RCC_OFF(PLL1DIVR) == 0x30, "RCC.PLL1DIVR");
_Static_assert(RCC_OFF(CSR) == 0x74, "RCC.CSR");
_Static_assert(RCC_OFF(RSR) == 0xD0, "RCC.RSR");
_Static_assert((__builtin_offsetof(PWR_Regs, CR3)) == 0x0C, "PWR.CR3");
_Static_assert((__builtin_offsetof(PWR_Regs, D3CR)) == 0x18, "PWR.D3CR");
#endif
