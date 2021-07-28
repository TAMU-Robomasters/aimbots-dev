/*
 * Copyright (c) 2011-2012, 2019, Fabian Greif
 * Copyright (c) 2012, 2015-2019, Niklas Hauser
 * Copyright (c) 2013, Sascha Schade
 * Copyright (c) 2013, 2015, Kevin Läufer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * ------------------------------------------------------------------------- */


MEMORY
{
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 2097152
	CCM (rw) : ORIGIN = 0x10000000, LENGTH = 65536
	SRAM1 (rwx) : ORIGIN = 0x20000000, LENGTH = 114688
	SRAM2 (rwx) : ORIGIN = 0x2001C000, LENGTH = 16384
	SRAM3 (rwx) : ORIGIN = 0x20020000, LENGTH = 65536
	BACKUP (rwx) : ORIGIN = 0x40024000, LENGTH = 4096
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 196608
    
}

OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)

/* First executable instruction in an output file */
ENTRY(Reset_Handler)

/* the thread stack is used only for reporting hard fault conditions! It may therefore be small. */
PROCESS_STACK_SIZE  = 0;
/* the handler stack is used for main program as well as interrupts */
MAIN_STACK_SIZE     = 3072;
/* combined stack size */
TOTAL_STACK_SIZE = MAIN_STACK_SIZE + PROCESS_STACK_SIZE;



SECTIONS
{
	__rom_start = ORIGIN(FLASH);


	.vector_rom :
	{
		__vector_table_rom_start = .;
		__vector_table_ram_load = .;
		/* Initial stack address, Reset and NMI handler */
		KEEP(*(.vector_rom))
		. = ALIGN(4);
	} >FLASH


	.vector_ram (NOLOAD) :
	{
		__vector_table_ram_start = .;
		/* Vector table in RAM, only if remapped */
		KEEP(*(.vector_ram))
		. = ALIGN(4);
		__vector_table_ram_end = .;
	} >RAM


	.stack (NOLOAD) :
	{
		__stack_start = .;

		. += MAIN_STACK_SIZE;
		. = ALIGN(8);
		__main_stack_top = .;

		. += PROCESS_STACK_SIZE;
		. = ALIGN(8);
		__process_stack_top = .;

		__stack_end = .;
	} >CCM


	.fastdata : ALIGN(4)
	{
		__fastdata_load = LOADADDR(.fastdata);
		__fastdata_start = .;
		*(.fastdata)
		. = ALIGN(4);
		__fastdata_end = .;
	} >CCM AT >FLASH


	.heap0 (NOLOAD) :
	{
		*(.heap0)
		. = ALIGN(4);
		__heap0_start = .;
		. = ORIGIN(CCM) + LENGTH(CCM);
		__heap0_end = .;
	} >CCM


	.text : ALIGN(4)
	{
		*(.text .text.* .gnu.linkonce.t.*)
	} >FLASH

	.rodata : ALIGN(4)
	{
		*(.rodata .rodata.* .gnu.linkonce.r.*)
		KEEP(*(.init))

		. = ALIGN(4);
		__init_array_start = .;
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		__init_array_end = .;
	} >FLASH

	/* Unwind tables are used to unwind the stack for C++ exceptions. Even
	 * though we disabled exceptions, pre-compiled libraries such as libstdc++
	 * still have to raise exceptions. We still force discard all tables
	 * though and hook these exception throwers in code. */
	/DISCARD/ :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		*(.eh_frame*)
	}
	/* Hardware init table */
	.hardware_init : ALIGN(4)
	{
		__hardware_init_start = .;
		/* Table symbols are alphabetically sorted! */
		KEEP(*(SORT(.hardware_init.order_*)))
		KEEP(*(SORT(.hardware_init)))
		__hardware_init_end = .;
	} >FLASH

	/* Assertion handler table */
	.assertion : ALIGN(4)
	{
		__assertion_table_start = .;
		KEEP(*(.assertion))
		__assertion_table_end = .;
	} >FLASH

	.build_id :
	{
		__build_id = .;
		KEEP(*(.note.gnu.build-id))
	} > FLASH


	.fastcode : ALIGN(4)
	{
		__fastcode_load = LOADADDR(.fastcode);
		__fastcode_start = .;
		*(.fastcode)
		. = ALIGN(4);
		__fastcode_end = .;
	} >FLASH


	/* initialized variables */
	.data : ALIGN(4)
	{
		__data_load = LOADADDR(.data);
		__data_start = .;
		*(.data .data.* .gnu.linkonce.d.*)
	} >RAM AT >FLASH

	/* code routines in RAM */
	.ramcode : ALIGN(4)
	{
		*(.ramcode .ramcode*)
		__data_end = .;
	} >RAM AT >FLASH

	/* uninitialized variables */
	.bss (NOLOAD) :
	{
		__bss_start = . ;
		*(.bss .bss.* .gnu.linkonce.b.*)
		. = ALIGN(4);
		__bss_end = .;
	} >RAM

	/* Global data not cleared after reset.  */
	.noinit (NOLOAD) :
	{
		__noinit_start = .;
		*(.noinit*)
		. = ALIGN(4);
		__noinit_end = .;
	} >RAM


	.heap1 (NOLOAD) :
	{
		*(.heap1)
		. = ALIGN(4);
		__heap1_start = .;
		. = ORIGIN(SRAM1) + LENGTH(SRAM1);
		__heap1_end = .;
	} >RAM


	.heap2 (NOLOAD) :
	{
		*(.heap2)
		. = ALIGN(4);
		__heap2_start = .;
		. = ORIGIN(SRAM2) + LENGTH(SRAM2);
		__heap2_end = .;
	} >RAM

	.heap3 (NOLOAD) :
	{
		*(.heap3)
		. = ALIGN(4);
		__heap3_start = .;
		. = ORIGIN(SRAM3) + LENGTH(SRAM3);
		__heap3_end = .;
	} >RAM

    

	/* TABLES! TABLES! ALL THE TABLES YOU COULD EVER WANT! TABLES! */
	.table.zero.intern : ALIGN(4)
	{
		__table_zero_intern_start = .;
		LONG(__bss_start)
		LONG(__bss_end)
		__table_zero_intern_end = .;
	} >FLASH


	.table.copy.intern : ALIGN(4)
	{
		__table_copy_intern_start = .;
		LONG(__data_load)
		LONG(__data_start)
		LONG(__data_end)
		LONG(__fastdata_load)
		LONG(__fastdata_start)
		LONG(__fastdata_end)
		__table_copy_intern_end = .;
	} >FLASH


	.table.zero.extern : ALIGN(4)
	{
		__table_zero_extern_start = .;
        
		__table_zero_extern_end = .;
	} >FLASH

	.table.copy.extern : ALIGN(4)
	{
		__table_copy_extern_start = .;
        
		__table_copy_extern_end = .;
	} >FLASH


	/* SRAM properties bit mask (16-bit):
	 *
	 * - 0x0001: accessible via S-Bus
	 * - 0x0002: accessible via D-Bus
	 * - 0x0004: accessible via I-Bus
	 * - 0x0008: accessible via DMA
	 * - 0x0010: accessible via DMA2D
	 *
	 * - 0x1FE0: reserved
	 *
	 * - 0x2000: Fast memory (Core- or Tightly-Coupled)
	 * - 0x4000: non-volatile (or battery backed) memory
	 * - 0x8000: external memory
	 */
	.table.heap : ALIGN(4)
	{
		__table_heap_start = .;
		LONG(0x001f)
		LONG(__heap1_start)
		LONG(__heap1_end)
		LONG(0x0019)
		LONG(__heap2_start)
		LONG(__heap2_end)
		LONG(0x0019)
		LONG(__heap3_start)
		LONG(__heap3_end)
		LONG(0x2002)
		LONG(__heap0_start)
		LONG(__heap0_end)
        
		__table_heap_end = .;
	} >FLASH


	. = ALIGN(4);
	__rom_end = .;


	/* DWARF debug sections */
	.debug_abbrev   0 : { *(.debug_abbrev) }
	.debug_aranges  0 : { *(.debug_aranges) }
	.debug_frame    0 : { *(.debug_frame) }
	.debug_info     0 : { *(.debug_info) }
	.debug_line     0 : { *(.debug_line) }
	.debug_loc      0 : { *(.debug_loc) }
	.debug_macinfo  0 : { *(.debug_macinfo) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	.debug_pubtypes 0 : { *(.debug_pubtypes) }
	.debug_ranges   0 : { *(.debug_ranges) }
	.debug_str      0 : { *(.debug_str) }

	.comment 0 : { *(.comment) }
	.ARM.attributes 0 : { KEEP(*(.ARM.attributes)) }
	/DISCARD/ : { *(.note.GNU-stack)  }

}