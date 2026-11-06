/*
 * board.h - bare-metal register map + board bring-up for the NUCLEO-H753ZI
 *
 * No HAL, no vendor headers: this file is the (small) subset of the STM32H753
 * reference manual (RM0433) that the servo firmware actually needs. Every
 * offset below is cross-checked against ST's CMSIS device header
 * (stm32h753xx.h) - see the static assertions at the bottom of board.c.
 *
 * Target: NUCLEO-H753ZI (STM32H753ZIT6, 2 MB flash, 1 MB RAM), 200 MHz SYSCLK
 * from the internal 64 MHz HSI oscillator through PLL1 (no crystal needed).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Core peripheral registers                                                  */
/* ------------------------------------------------------------------------- */
#define SCB_CPACR       (*(volatile uint32_t *)(0xE000ED88UL))  /* coprocessor access   */
#define DWT_CTRL        (*(volatile uint32_t *)(0xE0001000UL))  /* DWT control          */
#define DWT_CYCCNT      (*(volatile uint32_t *)(0xE0001004UL))  /* DWT cycle counter    */
#define DEMCR           (*(volatile uint32_t *)(0xE000EDFCUL))  /* debug exception/mon  */

/* ------------------------------------------------------------------------- */
/* Bus / peripheral base addresses (RM0433 "Memory map")                      */
/* ------------------------------------------------------------------------- */
#define PERIPH_BASE         0x40000000UL
#define D2_APB1PERIPH_BASE  (PERIPH_BASE + 0x00000000UL)          /* 0x4000_0000 */
#define D2_APB2PERIPH_BASE  (PERIPH_BASE + 0x00010000UL)          /* 0x4001_0000 */
#define D1_AHB1PERIPH_BASE  (PERIPH_BASE + 0x12000000UL)          /* 0x5200_0000 */
#define D3_AHB1PERIPH_BASE  (PERIPH_BASE + 0x18020000UL)          /* 0x5802_0000 */

#define FLASH_R_BASE        (D1_AHB1PERIPH_BASE + 0x00002000UL)   /* 0x5200_2000 */
#define TIM2_BASE           (D2_APB1PERIPH_BASE + 0x00000000UL)
#define TIM3_BASE           (D2_APB1PERIPH_BASE + 0x00000400UL)
#define TIM4_BASE           (D2_APB1PERIPH_BASE + 0x00000800UL)
#define TIM5_BASE           (D2_APB1PERIPH_BASE + 0x00000C00UL)
#define USART2_BASE         (D2_APB1PERIPH_BASE + 0x00004400UL)
#define USART3_BASE         (D2_APB1PERIPH_BASE + 0x00004800UL)
#define UART4_BASE          (D2_APB1PERIPH_BASE + 0x00004C00UL)
#define TIM1_BASE           (D2_APB2PERIPH_BASE + 0x00000000UL)
#define GPIOA_BASE          (D3_AHB1PERIPH_BASE + 0x00000000UL)
#define GPIOB_BASE          (D3_AHB1PERIPH_BASE + 0x00000400UL)
#define GPIOC_BASE          (D3_AHB1PERIPH_BASE + 0x00000800UL)
#define GPIOD_BASE          (D3_AHB1PERIPH_BASE + 0x00000C00UL)
#define GPIOE_BASE          (D3_AHB1PERIPH_BASE + 0x00001000UL)
#define GPIOF_BASE          (D3_AHB1PERIPH_BASE + 0x00001400UL)
#define GPIOG_BASE          (D3_AHB1PERIPH_BASE + 0x00001800UL)
#define RCC_BASE            (D3_AHB1PERIPH_BASE + 0x00004400UL)   /* 0x5802_4400 */
#define PWR_BASE            (D3_AHB1PERIPH_BASE + 0x00004800UL)   /* 0x5802_4800 */

/* ------------------------------------------------------------------------- */
/* Peripheral register layouts                                                */
/* ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t CR1;    /* 0x00 */
    volatile uint32_t CR2;    /* 0x04 */
    volatile uint32_t SMCR;   /* 0x08 */
    volatile uint32_t DIER;   /* 0x0C */
    volatile uint32_t SR;     /* 0x10 */
    volatile uint32_t EGR;    /* 0x14 */
    volatile uint32_t CCMR1;  /* 0x18  CH1 in [7:0], CH2 in [15:8]  */
    volatile uint32_t CCMR2;  /* 0x1C  CH3 in [7:0], CH4 in [15:8]  */
    volatile uint32_t CCER;   /* 0x20 */
    volatile uint32_t CNT;    /* 0x24 */
    volatile uint32_t PSC;    /* 0x28 */
    volatile uint32_t ARR;    /* 0x2C */
    volatile uint32_t RCR;    /* 0x30 */
    volatile uint32_t CCR1;   /* 0x34 */
    volatile uint32_t CCR2;   /* 0x38 */
    volatile uint32_t CCR3;   /* 0x3C */
    volatile uint32_t CCR4;   /* 0x40 */
    volatile uint32_t BDTR;   /* 0x44 */
} TIM_Regs;

typedef struct {
    volatile uint32_t CR1;    /* 0x00 */
    volatile uint32_t CR2;    /* 0x04 */
    volatile uint32_t CR3;    /* 0x08 */
    volatile uint32_t BRR;    /* 0x0C */
    volatile uint32_t GTPR;   /* 0x10 */
    volatile uint32_t RTOR;   /* 0x14 */
    volatile uint32_t RQR;    /* 0x18 */
    volatile uint32_t ISR;    /* 0x1C */
    volatile uint32_t ICR;    /* 0x20 */
    volatile uint32_t RDR;    /* 0x24 */
    volatile uint32_t TDR;    /* 0x28 */
} USART_Regs;

typedef struct {
    volatile uint32_t MODER;    /* 0x00 */
    volatile uint32_t OTYPER;   /* 0x04 */
    volatile uint32_t OSPEEDR;  /* 0x08 */
    volatile uint32_t PUPDR;    /* 0x0C */
    volatile uint32_t IDR;      /* 0x10 */
    volatile uint32_t ODR;      /* 0x14 */
    volatile uint32_t BSRR;     /* 0x18 */
    volatile uint32_t LCKR;     /* 0x1C */
    volatile uint32_t AFR[2];   /* 0x20 AFRL, 0x24 AFRH */
} GPIO_Regs;

typedef struct {
    volatile uint32_t ACR;        /* 0x00 */
    volatile uint32_t KEYR1;      /* 0x04 */
    volatile uint32_t OPTKEYR;    /* 0x08 */
    volatile uint32_t CR1;        /* 0x0C */
    volatile uint32_t SR1;        /* 0x10 */
    volatile uint32_t CCR1;       /* 0x14 */
    volatile uint32_t OPTCR;      /* 0x18 */
} FLASH_Regs;

typedef struct {
    volatile uint32_t CR1;    /* 0x00 */
    volatile uint32_t CSR1;   /* 0x04  ACTVOSRDY is bit 13 here */
    volatile uint32_t CR2;    /* 0x08 */
    volatile uint32_t CR3;    /* 0x0C  LDOEN / SMPSEN / SDEN / SCUEN */
    volatile uint32_t CPUCR;  /* 0x10 */
    volatile uint32_t RES1;   /* 0x14 */
    volatile uint32_t D3CR;   /* 0x18  VOS[15:14], VOSRDY bit 13 */
} PWR_Regs;

typedef struct {
    volatile uint32_t CR;          /* 0x00 */
    volatile uint32_t HSICFGR;     /* 0x04 */
    volatile uint32_t CRRCR;       /* 0x08 */
    volatile uint32_t CSICFGR;     /* 0x0C */
    volatile uint32_t CFGR;        /* 0x10 */
    volatile uint32_t RES1;        /* 0x14 */
    volatile uint32_t D1CFGR;      /* 0x18 */
    volatile uint32_t D2CFGR;      /* 0x1C */
    volatile uint32_t D3CFGR;      /* 0x20 */
    volatile uint32_t RES2;        /* 0x24 */
    volatile uint32_t PLLCKSELR;   /* 0x28 */
    volatile uint32_t PLLCFGR;     /* 0x2C */
    volatile uint32_t PLL1DIVR;    /* 0x30 */
    volatile uint32_t PLL1FRACR;   /* 0x34 */
    volatile uint32_t PLL2DIVR;    /* 0x38 */
    volatile uint32_t PLL2FRACR;   /* 0x3C */
    volatile uint32_t PLL3DIVR;    /* 0x40 */
    volatile uint32_t PLL3FRACR;   /* 0x44 */
    volatile uint32_t RES3;        /* 0x48 */
    volatile uint32_t D1CCIPR;     /* 0x4C */
    volatile uint32_t D2CCIP1R;    /* 0x50 */
    volatile uint32_t D2CCIP2R;    /* 0x54 */
    volatile uint32_t D3CCIPR;     /* 0x58 */
    volatile uint32_t RES4;        /* 0x5C */
    volatile uint32_t CIER;        /* 0x60 */
    volatile uint32_t CIFR;        /* 0x64 */
    volatile uint32_t CICR;        /* 0x68 */
    volatile uint32_t RES5;        /* 0x6C */
    volatile uint32_t BDCR;        /* 0x70 */
    volatile uint32_t CSR;         /* 0x74 */
    volatile uint32_t RES6;        /* 0x78 */
    volatile uint32_t AHB3RSTR;    /* 0x7C */
    volatile uint32_t AHB1RSTR;    /* 0x80 */
    volatile uint32_t AHB2RSTR;    /* 0x84 */
    volatile uint32_t AHB4RSTR;    /* 0x88 */
    volatile uint32_t APB3RSTR;    /* 0x8C */
    volatile uint32_t APB1LRSTR;   /* 0x90 */
    volatile uint32_t APB1HRSTR;   /* 0x94 */
    volatile uint32_t APB2RSTR;    /* 0x98 */
    volatile uint32_t APB4RSTR;    /* 0x9C */
    volatile uint32_t GCR;         /* 0xA0 */
    volatile uint32_t RES7;        /* 0xA4 */
    volatile uint32_t D3AMR;       /* 0xA8 */
    volatile uint32_t RES8[9];     /* 0xAC .. 0xCC */
    volatile uint32_t RSR;         /* 0xD0 */
} RCC_Regs;

/*
 * Clock-enable / reset registers live further up in the RCC block (0xD4..0xF4),
 * after the reset-status and enable registers. Accessing them as plain
 * addresses keeps RCC_Regs above short and unambiguous.
 */
#define RCC_AHB4ENR     (*(volatile uint32_t *)(RCC_BASE + 0xE0UL))
#define RCC_APB3ENR     (*(volatile uint32_t *)(RCC_BASE + 0xE4UL))
#define RCC_APB1LENR    (*(volatile uint32_t *)(RCC_BASE + 0xE8UL))
#define RCC_APB1HENR    (*(volatile uint32_t *)(RCC_BASE + 0xECUL))
#define RCC_APB2ENR     (*(volatile uint32_t *)(RCC_BASE + 0xF0UL))
#define RCC_APB4ENR     (*(volatile uint32_t *)(RCC_BASE + 0xF4UL))
#define RCC_BDCR        (*(volatile uint32_t *)(RCC_BASE + 0x70UL))
#define RCC_CSR         (*(volatile uint32_t *)(RCC_BASE + 0x74UL))
#define RCC_AHB4RSTR    (*(volatile uint32_t *)(RCC_BASE + 0x88UL))
#define RCC_APB1LRSTR   (*(volatile uint32_t *)(RCC_BASE + 0x90UL))
#define RCC_APB2RSTR    (*(volatile uint32_t *)(RCC_BASE + 0x98UL))
#define RCC_APB4RSTR    (*(volatile uint32_t *)(RCC_BASE + 0x9CUL))

/* ------------------------------------------------------------------------- */
/* Instances                                                                 */
/* ------------------------------------------------------------------------- */
#define TIM1    ((TIM_Regs *)TIM1_BASE)
#define TIM3    ((TIM_Regs *)TIM3_BASE)
#define USART3  ((USART_Regs *)USART3_BASE)
#define GPIOA   ((GPIO_Regs *)GPIOA_BASE)
#define GPIOB   ((GPIO_Regs *)GPIOB_BASE)
#define GPIOC   ((GPIO_Regs *)GPIOC_BASE)
#define GPIOD   ((GPIO_Regs *)GPIOD_BASE)
#define GPIOE   ((GPIO_Regs *)GPIOE_BASE)
#define GPIOF   ((GPIO_Regs *)GPIOF_BASE)
#define GPIOG   ((GPIO_Regs *)GPIOG_BASE)
#define FLASH   ((FLASH_Regs *)FLASH_R_BASE)
#define PWR     ((PWR_Regs *)PWR_BASE)
#define RCC     ((RCC_Regs *)RCC_BASE)

/* ------------------------------------------------------------------------- */
/* Bit definitions used by this firmware (RM0433 chapter numbers in comments) */
/* ------------------------------------------------------------------------- */

/* --- RCC ---------------------------------------------------------------- */
#define RCC_CR_HSION          (1UL << 0)
#define RCC_CR_HSIRDY         (1UL << 2)
#define RCC_CR_PLL1ON         (1UL << 24)
#define RCC_CR_PLL1RDY        (1UL << 25)

#define RCC_CFGR_SW_Pos       0
#define RCC_CFGR_SW_PLL1      (3UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SWS_Pos      3
#define RCC_CFGR_SWS_Msk      (7UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_PLL1     (3UL << RCC_CFGR_SWS_Pos)

#define RCC_D1CFGR_HPRE_Pos   0        /* AHB  prescaler  */
#define RCC_D1CFGR_D1CPRE_Pos 8        /* CPU  prescaler  */
#define RCC_D2CFGR_D2PPRE1_Pos 4       /* APB1 prescaler  */
#define RCC_D2CFGR_D2PPRE2_Pos 8       /* APB2 prescaler  */
#define RCC_D3CFGR_D3PPRE_Pos 4        /* APB4 prescaler  */
#define RCC_PRE_DIV1          0u
#define RCC_PRE_DIV2          4u       /* 0b100 = divide by 2 */

#define RCC_PLLCKSELR_PLLSRC_HSI  0u
#define RCC_PLLCKSELR_DIVM1_Pos   4

#define RCC_PLLCFGR_PLL1FRACEN    (1UL << 0)
#define RCC_PLLCFGR_PLL1VCOSEL    (1UL << 1)   /* 0 = wide VCO (192..836 MHz) */
#define RCC_PLLCFGR_PLL1RGE_Pos   2            /* 0b01 = 8..16 MHz ref clock  */
#define RCC_PLLCFGR_DIVP1EN       (1UL << 16)
#define RCC_PLLCFGR_DIVQ1EN       (1UL << 17)
#define RCC_PLLCFGR_DIVR1EN       (1UL << 18)

#define RCC_PLL1DIVR_PLL1P_Pos    0
#define RCC_PLL1DIVR_PLL1Q_Pos    8
#define RCC_PLL1DIVR_PLL1R_Pos    16
#define RCC_PLL1DIVR_PLL1N_Pos    24

#define RCC_AHB4ENR_GPIOAEN       (1UL << 0)
#define RCC_AHB4ENR_GPIOBEN       (1UL << 1)
#define RCC_AHB4ENR_GPIOCEN       (1UL << 2)
#define RCC_AHB4ENR_GPIODEN       (1UL << 3)
#define RCC_AHB4ENR_GPIOEEN       (1UL << 4)
#define RCC_AHB4ENR_GPIOFEN       (1UL << 5)
#define RCC_AHB4ENR_GPIOGEN       (1UL << 6)
#define RCC_APB1LENR_TIM3EN       (1UL << 1)
#define RCC_APB1LENR_USART3EN     (1UL << 18)
#define RCC_APB2ENR_TIM1EN        (1UL << 0)
#define RCC_APB4ENR_PWREN         (1UL << 1)

/* --- PWR ---------------------------------------------------------------- */
#define PWR_CR3_LDOEN             (1UL << 1)
#define PWR_CSR1_ACTVOSRDY        (1UL << 13)  /* voltage levels valid */
#define PWR_D3CR_VOS_Pos          14
#define PWR_D3CR_VOS1             (3UL << PWR_D3CR_VOS_Pos)

/* --- FLASH -------------------------------------------------------------- */
#define FLASH_ACR_LATENCY_Msk     0x0FUL
#define FLASH_ACR_LATENCY_2WS     2UL

/* --- GPIO --------------------------------------------------------------- */
#define GPIO_MODER_OUTPUT(pin)    (1UL << (2 * (pin)))
#define GPIO_MODER_AF(pin)        (2UL << (2 * (pin)))
#define GPIO_MODER_Msk(pin)       (3UL << (2 * (pin)))
#define GPIO_OSPEEDR_HIGH(pin)    (2UL << (2 * (pin)))
#define GPIO_PUPDR_NONE(pin)      (0UL << (2 * (pin)))

/* --- TIM ---------------------------------------------------------------- */
#define TIM_CR1_CEN               (1UL << 0)
#define TIM_CR1_ARPE              (1UL << 7)
#define TIM_EGR_UG                (1UL << 0)
#define TIM_CCMR_OCxM_PWM1        (6UL << 4)   /* PWM mode 1: CNT<CCR -> active */
#define TIM_CCMR_OCxPE            (1UL << 3)   /* preload enable               */
#define TIM_CCER_CCxE(ch)         (1UL << (4 * (ch)))
#define TIM_BDTR_MOE              (1UL << 15)  /* main output enable (adv. timers) */

/* --- USART -------------------------------------------------------------- */
#define USART_CR1_UE              (1UL << 0)
#define USART_CR1_TE              (1UL << 3)
#define USART_CR1_RE              (1UL << 2)
#define USART_ISR_RXNE            (1UL << 5)
#define USART_ISR_TXE             (1UL << 7)

/* ------------------------------------------------------------------------- */
/* Board configuration                                                       */
/* ------------------------------------------------------------------------- */
#define BOARD_SYSCLK_HZ     200000000UL   /* PLL1 output -> CPU clock        */
#define BOARD_HCLK_HZ       BOARD_SYSCLK_HZ
#define BOARD_PCLK1_HZ      (BOARD_HCLK_HZ / 2)   /* APB1, /2 -> 100 MHz    */
#define BOARD_PCLK2_HZ      (BOARD_HCLK_HZ / 2)   /* APB2, /2 -> 100 MHz    */
/* APB prescaler != 1 => timer kernel clock is 2 x PCLK */
#define BOARD_TIM1CLK_HZ    (BOARD_PCLK2_HZ * 2)  /* TIM1 on APB2: 200 MHz  */
#define BOARD_TIM3CLK_HZ    (BOARD_PCLK1_HZ * 2)  /* TIM3 on APB1: 200 MHz  */

#define BOARD_CONSOLE_BAUD  115200UL

/* Console UART: USART3 TX = PD8, RX = PD9 (ST-LINK virtual COM port). */
#define CONSOLE_USART       USART3
#define CONSOLE_TX_PORT     GPIOD
#define CONSOLE_TX_PIN      8
#define CONSOLE_RX_PORT     GPIOD
#define CONSOLE_RX_PIN      9

/* Alternate function numbers (RM0433 "AF mapping" tables) */
#define AF7_USART3          7u
#define AF1_TIM1            1u
#define AF2_TIM3            2u

/* ------------------------------------------------------------------------- */
/* Board API                                                                 */
/* ------------------------------------------------------------------------- */

/**
 * Regulator + flash wait states. MUST run before .data/.bss are initialised
 * (RAM writes are blocked until ACTVOSRDY is set), i.e. first thing in the
 * reset handler.
 */
void board_early_init(void);

/** 200 MHz from HSI via PLL1 (call after board_early_init()). */
void board_clock_init(void);

/** Set a pin to alternate-function push-pull mode. */
void board_gpio_af(GPIO_Regs *port, unsigned pin, unsigned af);

/** Set a pin to plain push-pull output. */
void board_gpio_output(GPIO_Regs *port, unsigned pin);

/** Enable the DWT cycle counter (used for delays and uptime). */
void board_dwt_init(void);

/** Blocking delay. */
void board_delay_ms(uint32_t ms);

/** Milliseconds since boot (wraps after ~58 days at 200 MHz / 200k cyc-ms). */
uint32_t board_millis(void);

/** Raw 32-bit timer value from a free-running TIM (used by nothing but debug). */
uint32_t board_cycles(void);
