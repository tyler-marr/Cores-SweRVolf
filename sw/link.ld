OUTPUT_ARCH( "riscv" )
ENTRY(_start)

MEMORY
{
  ram : ORIGIN = DEFINED(__ram_origin) ? __ram_origin : 0x80000000, LENGTH = 64k
}

SECTIONS
{
  . = 0;
  .text : { *(.text) }>ram
  .data : { *(.data) }>ram
  .bss : { *(.bss) }>ram
}
