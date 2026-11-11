/* STM32H743VIT6 on the WeAct MiniSTM32H7xx: code in flash bank 1, every RAM section in DTCM
   (board design 6: no DMA, so DTCM's DMA1/DMA2/BDMA restriction does not matter). */
ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
    DTCM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(DTCM) + LENGTH(DTCM);
_min_stack = 8K;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text .text.*)
        *(.rodata .rodata.*)
        KEEP(*(.build_info))
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH

    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data .data.*)
        . = ALIGN(4);
        _edata = .;
    } > DTCM AT > FLASH

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss .bss.* COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > DTCM

    /* No heap; the stack grows down from _estack and must keep _min_stack free. */
    ASSERT(_ebss + _min_stack <= _estack, "DTCM: no room for the stack")

    /DISCARD/ : { *(.ARM.attributes) }
}
