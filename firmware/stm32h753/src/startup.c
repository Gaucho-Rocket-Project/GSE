/*
 * startup.c - vector table, reset handler and C runtime bring-up
 *
 * The STM32H753 boots from 0x0800_0000 (the start of bank 1 flash) and reads
 * the initial stack pointer from offset 0x00 and the reset vector from 0x04.
 * The linker script puts .isr_vector there, so this table IS the boot image.
 */
#include <stdint.h>

#include "board.h"

/* Provided by the linker script */
extern uint32_t _sidata;    /* .data initial values, in flash           */
extern uint32_t _sdata;     /* .data start, in RAM                      */
extern uint32_t _edata;     /* .data end, in RAM                        */
extern uint32_t _sbss;      /* .bss start                               */
extern uint32_t _ebss;      /* .bss end                                 */
extern uint32_t _estack;    /* top of RAM = initial stack pointer       */

int  main(void);
void Reset_Handler(void);
void Default_Handler(void);

/* Every fault lands here: flash the red LED forever so a crash is visible. */
void Fault_Handler(void)
{
    /* LD3 (PB14) blink. Enable the port clock here rather than relying on
     * the rest of the firmware having got that far. */
    RCC_AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    (void)RCC_AHB4ENR;
    board_gpio_output(GPIOB, 14);
    for (;;) {
        GPIOB->BSRR = (1UL << 14);              /* on  */
        for (volatile uint32_t i = 0; i < 2000000UL; i++) {
        }
        GPIOB->BSRR = (1UL << (14 + 16));       /* off */
        for (volatile uint32_t i = 0; i < 2000000UL; i++) {
        }
    }
}

void Default_Handler(void)
{
    Fault_Handler();
}

/*
 * Vector table. The first 16 entries are the Cortex-M7 system exceptions;
 * the rest are the STM32H753 peripheral interrupts. Only Reset is used by
 * this firmware, so every other slot points at Default_Handler.
 */
#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))

void NMI_Handler(void)        WEAK_ALIAS;
void HardFault_Handler(void)  WEAK_ALIAS;
void MemManage_Handler(void)  WEAK_ALIAS;
void BusFault_Handler(void)   WEAK_ALIAS;
void UsageFault_Handler(void) WEAK_ALIAS;
void SVC_Handler(void)        WEAK_ALIAS;
void DebugMon_Handler(void)   WEAK_ALIAS;
void PendSV_Handler(void)     WEAK_ALIAS;
void SysTick_Handler(void)    WEAK_ALIAS;

extern uint32_t _estack;

__attribute__((section(".isr_vector"), used))
void (*const g_vectors[16])(void) = {
    (void (*)(void))&_estack,   /* 0x00 initial stack pointer   */
    Reset_Handler,              /* 0x04 reset                   */
    NMI_Handler,                /* 0x08                         */
    HardFault_Handler,          /* 0x0C                         */
    MemManage_Handler,          /* 0x10                         */
    BusFault_Handler,           /* 0x14                         */
    UsageFault_Handler,         /* 0x18                         */
    0,                          /* 0x1C reserved                */
    0,                          /* 0x20 reserved                */
    0,                          /* 0x24 reserved                */
    0,                          /* 0x28 reserved                */
    SVC_Handler,                /* 0x2C                         */
    DebugMon_Handler,           /* 0x30                         */
    0,                          /* 0x34 reserved                */
    PendSV_Handler,             /* 0x38                         */
    SysTick_Handler,            /* 0x3C                         */
};

/* Enable the FPU (CP10/CP11 full access) so the compiler may use VFP for the
 * odd float constant; this firmware never runs an FP context switch. */
static void fpu_enable(void)
{
    SCB_CPACR |= (0xFUL << 20);
    __asm volatile("dsb");
    __asm volatile("isb");
}

void Reset_Handler(void)
{
    /* 1. Put the vector table where the linker put it (flash bank 1). The
     *    reset default is 0x0000_0000, which is an alias of flash, but being
     *    explicit costs one register write. */
    *(volatile uint32_t *)0xE000ED08UL = 0x08000000UL;

    /* 2. FPU, then the regulator/flash setup that must happen before any RAM
     *    is written (see board_early_init). */
    fpu_enable();
    board_early_init();

    /* 3. Copy .data from flash to RAM, zero .bss. */
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }
    for (dst = &_sbss; dst < &_ebss; dst++) {
        *dst = 0;
    }

    (void)main();

    for (;;) {
        /* main() never returns; if it does, spin here. */
    }
}
