/*
 * Copyright (c) 2011-2012, 2019, Fabian Greif
 * Copyright (c) 2012, 2015-2022, Niklas Hauser
 * Copyright (c) 2013, Sascha Schade
 * Copyright (c) 2013, 2015, Kevin Läufer
 * Copyright (c) 2023, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * ------------------------------------------------------------------------- */


OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(Reset_Handler)

MEMORY
{
	ITCM (rwx) : ORIGIN = 0x00000000, LENGTH = 65536
	FLASH1 (rx) : ORIGIN = 0x08000000, LENGTH = 1048576
	DTCM (rwx) : ORIGIN = 0x20000000, LENGTH = 131072
	D1_SRAM (rwx) : ORIGIN = 0x24000000, LENGTH = 327680
	D2_SRAM1 (rwx) : ORIGIN = 0x30000000, LENGTH = 16384
	D2_SRAM2 (rwx) : ORIGIN = 0x30004000, LENGTH = 16384
	D3_SRAM (rwx) : ORIGIN = 0x38000000, LENGTH = 16384
	BACKUP (rwx) : ORIGIN = 0x38800000, LENGTH = 4096
	CONT_D2_SRAM1 (rwx) : ORIGIN = 0x30000000, LENGTH = 32768
}

__itcm_start = ORIGIN(ITCM);
__itcm_end = ORIGIN(ITCM) + LENGTH(ITCM);
__flash1_start = ORIGIN(FLASH1);
__flash1_end = ORIGIN(FLASH1) + LENGTH(FLASH1);
__dtcm_start = ORIGIN(DTCM);
__dtcm_end = ORIGIN(DTCM) + LENGTH(DTCM);
__d1_sram_start = ORIGIN(D1_SRAM);
__d1_sram_end = ORIGIN(D1_SRAM) + LENGTH(D1_SRAM);
__d2_sram1_start = ORIGIN(D2_SRAM1);
__d2_sram1_end = ORIGIN(D2_SRAM1) + LENGTH(D2_SRAM1);
__d2_sram2_start = ORIGIN(D2_SRAM2);
__d2_sram2_end = ORIGIN(D2_SRAM2) + LENGTH(D2_SRAM2);
__d3_sram_start = ORIGIN(D3_SRAM);
__d3_sram_end = ORIGIN(D3_SRAM) + LENGTH(D3_SRAM);
__backup_start = ORIGIN(BACKUP);
__backup_end = ORIGIN(BACKUP) + LENGTH(BACKUP);


MAIN_STACK_SIZE = 3072;
EXCEPTION_FRAME_SIZE = 108;

SECTIONS
{
	__rom_start = ORIGIN(FLASH1);


	/* Read-only vector table in FLASH1 */
	.text :
	{
		__vector_table_rom_start = .;
		__vector_table_ram_load = .;
		KEEP(*(.vector_rom))
		. = ALIGN(8);
		__vector_table_rom_end = .;
	} >FLASH1


	/* build id directly after vector table */
	.build_id :
	{
		__build_id = .;
		KEEP(*(.note.gnu.build-id))
	} >FLASH1

	/* Read-only sections in FLASH1 */
	.text :
	{
		*(.text .text.* .gnu.linkonce.t.*)
	} >FLASH1

	.rodata :
	{
		*(.rodata .rodata.* .gnu.linkonce.r.*)
		KEEP(*(.init))

		. = ALIGN(4);
		__init_array_start = .;
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		. = ALIGN(4);
		__init_array_end = .;

		__hardware_init_start = .;
		KEEP(*(SORT(.hardware_init.order_*)))
		KEEP(*(SORT(.hardware_init)))
		. = ALIGN(4);
		__hardware_init_end = .;

		__assertion_table_start = .;
		KEEP(*(.assertion))
		. = ALIGN(4);
		__assertion_table_end = .;
	} >FLASH1

	/DISCARD/ :
	{
		/* We do not call static destructors ever */
		*(.fini_array .fini_array.*)
		/* C++ exception unwind tables are discarded */
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		*(.eh_frame*)
	}


	/* Read-Write vector table in ITCM */
	.vectors (NOLOAD) :
	{
		__vector_table_ram_start = .;
		KEEP(*(.vector_ram))
		. = ALIGN(8);
		__vector_table_ram_end = .;
	} >ITCM

	/* Sections in memory region ITCM */

	.fastcode :
	{
		. = ALIGN(8);
		__fastcode_load = LOADADDR(.fastcode);
		__fastcode_start = .;
		*(.fastcode .fastcode.*)
		. = ALIGN(8);
		__fastcode_end = .;
	} >ITCM AT >FLASH1

	.data_itcm :
	{
		. = ALIGN(8);
		__data_itcm_load = LOADADDR(.data_itcm);
		__data_itcm_start = .;
		*(.data_itcm .data_itcm.*)
		. = ALIGN(8);
		__data_itcm_end = .;
	} >ITCM AT >FLASH1


	.noinit_itcm (NOLOAD) :
	{
		. = ALIGN(4);
		__noinit_itcm_start = .;
		*(.noinit_itcm .noinit_itcm.*)
		. = ALIGN(4);
		__noinit_itcm_end = .;
	} >ITCM

	.heap_itcm (NOLOAD) :
	{
		. = ALIGN(4);
		__heap_itcm_start = .;
		. = MAX(ABSOLUTE(.), ORIGIN(ITCM) + LENGTH(ITCM));
		__heap_itcm_end = .;
	} >ITCM

	/* Main stack in D1_SRAM */
	.stack (NOLOAD) :
	{
		__stack_start = .;
		__main_stack_bottom = .;
		/* main stack must fit one entire exception after the limit */
		__main_stack_limit = . + EXCEPTION_FRAME_SIZE;

		. += MAIN_STACK_SIZE;
		. = ALIGN(8);
		__main_stack_top = .;

		__stack_end = .;
	} >D1_SRAM

	/* Read-write sections in D1_SRAM */
	.data :
	{
		. = ALIGN(8);
		__data_load = LOADADDR(.data);
		__data_start = .;
		*(.data .data.* .gnu.linkonce.d.*)
		. = ALIGN(8);
		__data_end = .;
	} >D1_SRAM AT >FLASH1

	.data_d1_sram :
	{
		__data_d1_sram_load = LOADADDR(.data_d1_sram);
		__data_d1_sram_start = .;
		*(.data_d1_sram .data_d1_sram.*)
		. = ALIGN(8);
		__data_d1_sram_end = .;
	} >D1_SRAM AT >FLASH1

	.bss (NOLOAD) :
	{
		__bss_start = . ;
		*(.bss .bss.* .gnu.linkonce.b.*)
		. = ALIGN(4);
	} >D1_SRAM

	.bss_d1_sram (NOLOAD) :
	{
		__bss_d1_sram_start = . ;
		*(.bss_d1_sram .bss_d1_sram.*)
		. = ALIGN(4);
		__bss_d1_sram_end = .;
		. = ALIGN(8);
		__bss_end = .;
	} >D1_SRAM


	.noinit (NOLOAD) :
	{
		__noinit_start = . ;
		*(.noinit .noinit.*)
		. = ALIGN(8);
		__noinit_end = .;
	} >D1_SRAM

	.faststack (NOLOAD) :
	{
		__faststack_start = . ;
		*(.faststack .faststack.*)
		. = ALIGN(8);
		__faststack_end = .;
	} >D1_SRAM


	.fastdata :
	{
		. = ALIGN(8);
		__fastdata_load = LOADADDR(.fastdata);
		__fastdata_start = .;
		*(.fastdata .fastdata.*)
		. = ALIGN(8);
		__fastdata_end = .;
	} >DTCM AT >FLASH1

	/* Sections in DTCM */

	.data_dtcm :
	{
		. = ALIGN(8);
		__data_dtcm_load = LOADADDR(.data_dtcm);
		__data_dtcm_start = .;
		*(.data_dtcm .data_dtcm.*)
		. = ALIGN(8);
		__data_dtcm_end = .;
	} >DTCM AT >FLASH1

	.bss_dtcm (NOLOAD) :
	{
		. = ALIGN(4);
		__bss_dtcm_start = .;
		*(.bss_dtcm .bss_dtcm.*)
		. = ALIGN(4);
		__bss_dtcm_end = .;
	} >DTCM

	.noinit_dtcm (NOLOAD) :
	{
		. = ALIGN(4);
		__noinit_dtcm_start = .;
		*(.noinit_dtcm .noinit_dtcm.*)
		. = ALIGN(4);
		__noinit_dtcm_end = .;
	} >DTCM

	.heap_dtcm (NOLOAD) :
	{
		. = ALIGN(4);
		__heap_dtcm_start = .;
		. = MAX(ABSOLUTE(.), ORIGIN(DTCM) + LENGTH(DTCM));
		__heap_dtcm_end = .;
	} >DTCM


	/* Sections in D1_SRAM */
	.noinit_d1_sram (NOLOAD) :
	{
		. = ALIGN(4);
		__noinit_d1_sram_start = .;
		*(.noinit_d1_sram .noinit_d1_sram.*)
		. = ALIGN(4);
		__noinit_d1_sram_end = .;
	} >D1_SRAM

	.heap_d1_sram (NOLOAD) :
	{
		. = ALIGN(4);
		__heap_d1_sram_start = .;
		. = MAX(ABSOLUTE(.), ORIGIN(D1_SRAM) + LENGTH(D1_SRAM));
		__heap_d1_sram_end = .;
	} >D1_SRAM


	/* Sections in D2_SRAM1 */

	.data_d2_sram1 :
	{
		. = ALIGN(8);
		__data_d2_sram1_load = LOADADDR(.data_d2_sram1);
		__data_d2_sram1_start = .;
		*(.data_d2_sram1 .data_d2_sram1.*)
		. = ALIGN(8);
		__data_d2_sram1_end = .;
	} >CONT_D2_SRAM1 AT >FLASH1

	.bss_d2_sram1 (NOLOAD) :
	{
		. = ALIGN(4);
		__bss_d2_sram1_start = .;
		*(.bss_d2_sram1 .bss_d2_sram1.*)
		. = ALIGN(4);
		__bss_d2_sram1_end = .;
	} >CONT_D2_SRAM1

	.noinit_d2_sram1 (NOLOAD) :
	{
		. = ALIGN(4);
		__noinit_d2_sram1_start = .;
		*(.noinit_d2_sram1 .noinit_d2_sram1.*)
		. = ALIGN(4);
		__noinit_d2_sram1_end = .;
	} >CONT_D2_SRAM1

	.heap_d2_sram1 (NOLOAD) :
	{
		. = ALIGN(4);
		__heap_d2_sram1_start = .;
		. = MAX(ABSOLUTE(.), ORIGIN(D2_SRAM1) + LENGTH(D2_SRAM1));
		__heap_d2_sram1_end = .;
	} >CONT_D2_SRAM1


	/* Sections in D2_SRAM2 */

	.data_d2_sram2 :
	{
		. = ALIGN(8);
		__data_d2_sram2_load = LOADADDR(.data_d2_sram2);
		__data_d2_sram2_start = .;
		*(.data_d2_sram2 .data_d2_sram2.*)
		. = ALIGN(8);
		__data_d2_sram2_end = .;
	} >CONT_D2_SRAM1 AT >FLASH1

	.bss_d2_sram2 (NOLOAD) :
	{
		. = ALIGN(4);
		__bss_d2_sram2_start = .;
		*(.bss_d2_sram2 .bss_d2_sram2.*)
		. = ALIGN(4);
		__bss_d2_sram2_end = .;
	} >CONT_D2_SRAM1

	.noinit_d2_sram2 (NOLOAD) :
	{
		. = ALIGN(4);
		__noinit_d2_sram2_start = .;
		*(.noinit_d2_sram2 .noinit_d2_sram2.*)
		. = ALIGN(4);
		__noinit_d2_sram2_end = .;
	} >CONT_D2_SRAM1

	.heap_d2_sram2 (NOLOAD) :
	{
		. = ALIGN(4);
		__heap_d2_sram2_start = .;
		. = MAX(ABSOLUTE(.), ORIGIN(D2_SRAM2) + LENGTH(D2_SRAM2));
		__heap_d2_sram2_end = .;
	} >CONT_D2_SRAM1


	/* Sections in D3_SRAM */

	.data_d3_sram :
	{
		. = ALIGN(8);
		__data_d3_sram_load = LOADADDR(.data_d3_sram);
		__data_d3_sram_start = .;
		*(.data_d3_sram .data_d3_sram.*)
		. = ALIGN(8);
		__data_d3_sram_end = .;
	} >D3_SRAM AT >FLASH1

	.bss_d3_sram (NOLOAD) :
	{
		. = ALIGN(4);
		__bss_d3_sram_start = .;
		*(.bss_d3_sram .bss_d3_sram.*)
		. = ALIGN(4);
		__bss_d3_sram_end = .;
	} >D3_SRAM

	.noinit_d3_sram (NOLOAD) :
	{
		. = ALIGN(4);
		__noinit_d3_sram_start = .;
		*(.noinit_d3_sram .noinit_d3_sram.*)
		. = ALIGN(4);
		__noinit_d3_sram_end = .;
	} >D3_SRAM

	.heap_d3_sram (NOLOAD) :
	{
		. = ALIGN(4);
		__heap_d3_sram_start = .;
		. = MAX(ABSOLUTE(.), ORIGIN(D3_SRAM) + LENGTH(D3_SRAM));
		__heap_d3_sram_end = .;
	} >D3_SRAM



	/* Sections in memory region BACKUP */

	.data_backup :
	{
		. = ALIGN(8);
		__data_backup_load = LOADADDR(.data_backup);
		__data_backup_start = .;
		*(.data_backup .data_backup.*)
		. = ALIGN(8);
		__data_backup_end = .;
	} >BACKUP AT >FLASH1


	.bss_backup (NOLOAD) :
	{
		. = ALIGN(4);
		__bss_backup_start = .;
		*(.bss_backup .bss_backup.*)
		. = ALIGN(4);
		__bss_backup_end = .;
	} >BACKUP

	.noinit_backup (NOLOAD) :
	{
		. = ALIGN(4);
		__noinit_backup_start = .;
		*(.noinit_backup .noinit_backup.*)
		. = ALIGN(4);
		__noinit_backup_end = .;
	} >BACKUP

	.heap_backup (NOLOAD) :
	{
		. = ALIGN(4);
		__heap_backup_start = .;
		. = MAX(ABSOLUTE(.), ORIGIN(BACKUP) + LENGTH(BACKUP));
		__heap_backup_end = .;
	} >BACKUP

	/* Memory layout configuration tables */
	.rodata :
	{
		. = ALIGN(4);
		__table_zero_intern_start = .;
		LONG(__bss_start)
		LONG(__bss_end)
		LONG(__bss_dtcm_start)
		LONG(__bss_dtcm_end)
		LONG(__bss_d2_sram1_start)
		LONG(__bss_d2_sram1_end)
		LONG(__bss_d2_sram2_start)
		LONG(__bss_d2_sram2_end)
		LONG(__bss_d3_sram_start)
		LONG(__bss_d3_sram_end)
		LONG(__bss_backup_start)
		LONG(__bss_backup_end)
		__table_zero_intern_end = .;

		__table_copy_intern_start = .;
		LONG(__vector_table_ram_load)
		LONG(__vector_table_ram_start)
		LONG(__vector_table_ram_end)
		LONG(__fastcode_load)
		LONG(__fastcode_start)
		LONG(__fastcode_end)
		LONG(__data_itcm_load)
		LONG(__data_itcm_start)
		LONG(__data_itcm_end)
		LONG(__data_load)
		LONG(__data_start)
		LONG(__data_end)
		LONG(__data_d1_sram_load)
		LONG(__data_d1_sram_start)
		LONG(__data_d1_sram_end)
		LONG(__fastdata_load)
		LONG(__fastdata_start)
		LONG(__fastdata_end)
		LONG(__data_dtcm_load)
		LONG(__data_dtcm_start)
		LONG(__data_dtcm_end)
		LONG(__data_d2_sram1_load)
		LONG(__data_d2_sram1_start)
		LONG(__data_d2_sram1_end)
		LONG(__data_d2_sram2_load)
		LONG(__data_d2_sram2_start)
		LONG(__data_d2_sram2_end)
		LONG(__data_d3_sram_load)
		LONG(__data_d3_sram_start)
		LONG(__data_d3_sram_end)
		LONG(__data_backup_load)
		LONG(__data_backup_start)
		LONG(__data_backup_end)
		__table_copy_intern_end = .;

		__table_zero_extern_start = .;
		__table_zero_extern_end = .;

		__table_copy_extern_start = .;
		__table_copy_extern_end = .;

		/* See `modm:architecture:memory` for bitmask */
		__table_heap_start = .;
		LONG(0x2005)
		LONG(__heap_itcm_start)
		LONG(__heap_itcm_end)
		LONG(0x2023)
		LONG(__heap_dtcm_start)
		LONG(__heap_dtcm_end)
		LONG(0x001f)
		LONG(__heap_d1_sram_start)
		LONG(__heap_d1_sram_end)
		LONG(0x001f)
		LONG(__heap_d2_sram1_start)
		LONG(__heap_d2_sram1_end)
		LONG(0x001f)
		LONG(__heap_d2_sram2_start)
		LONG(__heap_d2_sram2_end)
		LONG(0x001f)
		LONG(__heap_d3_sram_start)
		LONG(__heap_d3_sram_end)
		LONG(0x4009)
		LONG(__heap_backup_start)
		LONG(__heap_backup_end)
		__table_heap_end = .;
	} >FLASH1


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