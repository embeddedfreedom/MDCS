/******************************************************************************
 *
 * Linker configuration file for usb_dev_keyboard.
 *
 *
 *****************************************************************************/

MEMORY
{
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 0x00080000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00020000
}

SECTIONS
{
    .text :
    {
        _text = .;
        *(.isr_vector)
        *(.text*)
        *(.rodata*)
        *(.ARM.exidx*)
        *(.eh_frame*)
        _etext = .;
    } > FLASH

    .data : AT(_etext)
    {
        _data = .;
        *(.data*)
        _edata = .;
    } > SRAM

    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM
}
