/*
 * link.ld - NUCLEO-H753ZI (STM32H753ZIT6: 2 MB flash, 1 MB RAM)
 *
 *   FLASH 0x0800_0000 .. 0x081F_FFFF   (bank 1 / bank 2, 2 MB total)
 *   DTCM  0x2000_0000 .. 0x2001_FFFF   (128 KB, fastest, DMA cannot reach it)
 *   AXI SRAM 0x2400_0000 .. 0x2407_FFFF (512 KB, default .data/.bss/.stack)
 *   SRAM1-3 0x3000_0000 .. 0x3004_7FFF (288 KB)
 *
 * The image is linked as a single bank-1 image at 0x0800_0000: that is what
 * `openocd -c "program firmware.elf"` and the ST-LINK drag-and-drop .bin
 * expect, and it leaves the whole of the second bank free.
 *
 * Stack lives at the top of AXI SRAM so it never collides with .bss.
 */
ENTRY(Reset_Handler)

MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
    DTCM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    AXI    (rwx) : ORIGIN = 0x24000000, LENGTH = 512K
    SRAM   (rwx) : ORIGIN = 0x30000000, LENGTH = 288K
}

/* Stack: 16 KB at the top of AXI SRAM (grows down). */
_stack_size = 0x4000;
_estack     = ORIGIN(AXI) + LENGTH(AXI);

SECTIONS
{
    /* Vector table must be the very first thing in flash. */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
        . = ALIGN(4);
    } > FLASH

    .ARM.exidx :
    {
        *(.ARM.exidx*)
    } > FLASH

    /* Initialised data: lives in RAM, copied from flash by Reset_Handler. */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > AXI AT> FLASH
    _sidata = LOADADDR(.data);

    /* Zero-initialised data. */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > AXI

    /*
     * RAM that is NOT initialised at boot: survives a soft reset, so the fault
     * handler's report survives the reset it triggers. Cleared only by the
     * bootloader/ST-LINK "reset" with mass erase, or a power cycle.
     */
    .noinit (NOLOAD) :
    {
        . = ALIGN(8);
        KEEP(*(.noinit))
        KEEP(*(.noinit*))
        . = ALIGN(8);
    } > AXI

    /* Stack reservation + overflow guard marker. */
    ._stack (NOLOAD) :
    {
        . = ALIGN(8);
        _sstack = .;
        . = . + _stack_size;
        . = ALIGN(8);
    } > AXI

    end = .;
    _end = .;
    __end__ = .;

    /DISCARD/ :
    {
        *(.note*)
        *(.comment)
        *(.eh_frame*)
    }
}

ASSERT(_sstack + _stack_size <= _estack, "stack overlaps top of RAM")
ASSERT(_ebss <= _sstack, "bss runs into the stack")
