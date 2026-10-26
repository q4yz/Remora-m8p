/* m8p_probe: one copy, linked to run at 0x08010000, at most 4 KB minus the ID word */
MEMORY
{
    PROBE (rx) : ORIGIN = 0x08010000, LENGTH = 0x0FFC
}

SECTIONS
{
    .text : { KEEP(*(.entry)) *(.text*) *(.rodata*) . = ALIGN(4); } > PROBE
    /DISCARD/ : { *(.data*) *(.bss*) *(COMMON) *(.ARM.*) *(.comment) }
}
