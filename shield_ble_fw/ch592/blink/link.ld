MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 448K
	RAM (rw)   : ORIGIN = 0x20000000, LENGTH = 26K
}

SECTIONS
{
	.text : {
		KEEP(*(.init))
		*(.text*)
		*(.rodata*)
	} > FLASH
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
