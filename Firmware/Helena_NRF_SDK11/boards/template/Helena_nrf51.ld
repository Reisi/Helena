/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __etext
 *   __noinit_start
 *   __noinit_end
 *   __text_start
 *   __text_end
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 */


OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
ENTRY(Reset_Handler)

MEMORY
{
  SOFTD (rx)        : ORIGIN = 0x00000000, LENGTH = 0x0001B000  /* flash used by softdevice */
  FLASH (rx)        : ORIGIN = 0x0001B000, LENGTH = 0x00020000  /* flash available for application (with single bank bootloader) */
  FLASH_USERDATA (r): ORIGIN = 0x0003B000, LENGTH = 0x00001000  /* flash for user data (peer manager, ...) */
  FLASH_BTLDR (r)   : ORIGIN = 0x0003C000, LENGTH = 0x00003C00  /* flash used for bootloader */
  FLASH_BTLD_SET (r): ORIGIN = 0x0003FC00, LENGTH = 0x00000400  /* flash used for bootloader settings */
  RAM_SOFTD (r)     : ORIGIN = 0x20000000, LENGTH = 0x00002AF8  /* RAM used by softdevice */
  RAM (rwx)         : ORIGIN = 0x20002A60, LENGTH = 0x00001508  /* RAM available for application */
  NOINIT (rwx)      : ORIGIN = 0x20003F68, LENGTH = 0x00000058  /* non initialized RAM, when changed check against bootloader ram usage! */
  NOINIT_BTLDR (rwx): ORIGIN = 0x20003FC0, LENGTH = 0x00000040  /* non initialized RAM used to exchange bonding information with bootloader */
}

SECTIONS
{
    /* No init RAM section. */
    .noinit(NOLOAD) :
    {
        PROVIDE(__noinit_start = .);
        KEEP(*(.noinit))
        PROVIDE(__noinit_end = .);
    } > NOINIT

    /* The normal application area of the device with SoftDevice
    */
	.text :
	{
		PROVIDE(__text_start = .);

		KEEP(*(.Vectors))
		*(.text*)

		KEEP(*(.init))
		KEEP(*(.fini))

		/* .ctors */
		*crtbegin.o(.ctors)
		*crtbegin?.o(.ctors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
		*(SORT(.ctors.*))
		*(.ctors)

		/* .dtors */
 		*crtbegin.o(.dtors)
 		*crtbegin?.o(.dtors)
 		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
 		*(SORT(.dtors.*))
 		*(.dtors)

		*(.rodata*)

		*(.eh_frame*)
		. = ALIGN(4);

		PROVIDE(__text_end = .);
	} > FLASH

	/* section variables */
  .debug_commands :
  {
    PROVIDE(__start_debug_commands = .);
    KEEP(*(.debug_commands))
    PROVIDE(__stop_debug_commands = .);
  } > FLASH

  .sys_events :
  {
    PROVIDE(__start_sys_events = .);
    KEEP(*(.sys_events))
    PROVIDE(__stop_sys_events = .);
  } > FLASH

  .btle_services :
  {
    PROVIDE(__start_btle_services = .);
    KEEP(*(.btle_services))
    PROVIDE(__stop_btle_services = .);
  } > FLASH

  .remote_driver :
  {
    PROVIDE(__start_remote_driver = .);
    KEEP(*(.remote_driver))
    PROVIDE(__stop_remote_driver = .);
  } > FLASH

	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		. = ALIGN(4);
	} > FLASH

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		. = ALIGN(4);
	} > FLASH
	__exidx_end = .;

	__etext = .;

	.data : AT (__etext)
	{
		__data_start__ = .;
		*(vtable)
		*(.data*)

		. = ALIGN(4);
		/* preinit data */
		PROVIDE_HIDDEN (__preinit_array_start = .);
		*(.preinit_array)
		PROVIDE_HIDDEN (__preinit_array_end = .);

		. = ALIGN(4);
		/* init data */
		PROVIDE_HIDDEN (__init_array_start = .);
		*(SORT(.init_array.*))
		*(.init_array)
		PROVIDE_HIDDEN (__init_array_end = .);


		. = ALIGN(4);
		/* finit data */
		PROVIDE_HIDDEN (__fini_array_start = .);
		*(SORT(.fini_array.*))
		*(.fini_array)
		PROVIDE_HIDDEN (__fini_array_end = .);

		*(.jcr)
		. = ALIGN(4);
		/* All data end */
		__data_end__ = .;

	} > RAM

	.fs_data :
    {
        . = ALIGN(4);
        PROVIDE(__start_fs_data = .);
        KEEP(*(.fs_data))
        PROVIDE(__stop_fs_data = .);
        . = ALIGN(4);
        __data_end__ = .;
    } > RAM

	.bss :
	{
		. = ALIGN(4);
		__bss_start__ = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		__bss_end__ = .;
	} > RAM

	.heap (COPY):
	{
		__end__ = .;
		end = __end__;
		*(.heap*)
		__HeapLimit = .;
	} > RAM

	/* .stack_dummy section doesn't contains any symbols. It is only
	 * used for linker to calculate size of stack sections, and assign
	 * values to stack symbols later */
	.stack_dummy (COPY):
	{
		*(.stack*)
	} > RAM

	/* Set stack top to end of RAM, and stack limit move down by
	 * size of stack_dummy section */
	__StackTop = ORIGIN(RAM) + LENGTH(RAM);
	__StackLimit = __StackTop - SIZEOF(.stack_dummy);
	PROVIDE(__stack = __StackTop);

	/* Check if data + heap + stack exceeds RAM limit */
	ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
}


