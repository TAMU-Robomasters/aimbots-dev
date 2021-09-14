/*
 * Copyright (c) 2011-2012, 2019, Fabian Greif
 * Copyright (c) 2012, 2015-2021, Niklas Hauser
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
	CONT_SRAM1 (rwx) : ORIGIN = 0x20000000, LENGTH = 196608
    
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
__ccm_start = ORIGIN(CCM);
__ccm_end = ORIGIN(CCM) + LENGTH(CCM);
__sram1_start = ORIGIN(SRAM1);
__sram1_end = ORIGIN(SRAM1) + LENGTH(SRAM1);
__sram2_start = ORIGIN(SRAM2);
__sram2_end = ORIGIN(SRAM2) + LENGTH(SRAM2);
__sram3_start = ORIGIN(SRAM3);
__sram3_end = ORIGIN(SRAM3) + LENGTH(SRAM3);
__backup_start = ORIGIN(BACKUP);
__backup_end = ORIGIN(BACKUP) + LENGTH(BACKUP);
OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(Reset_Handler)

/* the handler stack is used for main program as well as interrupts */
MAIN_STACK_SIZE    = 3072;
PROCESS_STACK_SIZE = 0;
TOTAL_STACK_SIZE   = MAIN_STACK_SIZE + PROCESS_STACK_SIZE;

SECTIONS
{
	__rom_start = ORIGIN(FLASH);


	.text :
	{
		__vector_table_rom_start = .;
		__vector_table_ram_load = .;
		/* Initial stack address, Reset and NMI handler */
		KEEP(*(.vector_rom))
	} >FLASH


	.text :
	{
		*(.text .text.* .gnu.linkonce.t.*)
	} >FLASH

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
		/* Table symbols are alphabetically sorted! */
		KEEP(*(SORT(.hardware_init.order_*)))
		KEEP(*(SORT(.hardware_init)))
		. = ALIGN(4);
		__hardware_init_end = .;

		__assertion_table_start = .;
		KEEP(*(.assertion))
		. = ALIGN(4);
		__assertion_table_end = .;

		__build_id = .;
		KEEP(*(.note.gnu.build-id))
	} >FLASH

	/* Unwind tables are used to unwind the stack for C++ exceptions. Even
	 * though we disabled exceptions, pre-compiled libraries such as libstdc++
	 * still have to raise exceptions. */
	/DISCARD/ :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		*(.eh_frame*)
	}



	.data :
	{
		. = ALIGN(4);
		__data_load = LOADADDR(.data);
		__data_start = .;
		*(.data .data.* .gnu.linkonce.d.*)
		. = ALIGN(4);
	} >CONT_SRAM1 AT >FLASH

	.fastcode :
	{
		__fastcode_load = LOADADDR(.fastcode);
		__fastcode_start = .;
		*(.fastcode .fastcode.*)
		. = ALIGN(4);
		__fastcode_end = .;
	} >CONT_SRAM1 AT >FLASH
	.data_sram1 :
	{
		__data_sram1_load = LOADADDR(.data_sram1);
		__data_sram1_start = .;
		*(.data_sram1 .data_sram1.*)
		. = ALIGN(4);
		__data_sram1_end = .;
	} >CONT_SRAM1 AT >FLASH
	__data_end = .;

	.bss (NOLOAD) :
	{
		__bss_start = . ;
		*(.bss .bss.* .gnu.linkonce.b.*)
		. = ALIGN(4);
		__bss_end = .;
	} >CONT_SRAM1

	.bss_sram1 (NOLOAD) :
	{
		__bss_sram1_start = . ;
		*(.bss_sram1 .bss_sram1.*)
		. = ALIGN(4);
		__bss_sram1_end = .;
	} >CONT_SRAM1
	__bss_end = .;

	.noinit (NOLOAD) :
	{
		__noinit_start = .;
		*(.noinit .noinit.*)
		. = ALIGN(4);
		__noinit_end = .;
	} >CONT_SRAM1


	.noinit_sram1 (NOLOAD) :
	{
		. = ALIGN(4);
		__noinit_sram1_start = .;
		*(.noinit_sram1 .noinit_sram1.*)
		. = ALIGN(4);
		__noinit_sram1_end = .;
	} >CONT_SRAM1
	.heap_sram1 (NOLOAD) :
	{
		. = ALIGN(4);
		__heap_sram1_start = .;
		. = MAX(ABSOLUTE(.), ORIGIN(SRAM1) + LENGTH(SRAM1));
		__heap_sram1_end = .;
	} >CONT_SRAM1

	.data_sram2 :
	{
		. = ALIGN(4);
		__data_sram2_load = LOADADDR(.data_sram2);
		__data_sram2_start = .;
		*(.data_sram2 .data_sram2.*)
		. = ALIGN(4);
	} >CONT_SRAM1 AT >FLASH

	__data_sram2_end = .;

	.bss_sram2 (NOLOAD) :
	{
		. = ALIGN(4);
		__bss_sram2_start = .;
		*(.bss_sram2 .bss_sram2.*)
		. = ALIGN(4);
		__bss_sram2_end = .;
	} >CONT_SRAM1
	.noinit_sram2 (NOLOAD) :
	{
		. = ALIGN(4);
		__noinit_sram2_start = .;
		*(.noinit_sram2 .noinit_sram2.*)
		. = ALIGN(4);
		__noinit_sram2_end = .;
	} >CONT_SRAM1
	.heap_sram2 (NOLOAD) :
	{
		. = ALIGN(4);
		__heap_sram2_start = .;
		. = MAX(ABSOLUTE(.), ORIGIN(SRAM2) + LENGTH(SRAM2));
		__heap_sram2_end = .;
	} >CONT_SRAM1

	.data_sram3 :
	{
		. = ALIGN(4);
		__data_sram3_load = LOADADDR(.data_sram3);
		__data_sram3_start = .;
		*(.data_sram3 .data_sram3.*)
		. = ALIGN(4);
	} >CONT_SRAM1 AT >FLASH

	__data_sram3_end = .;

	.bss_sram3 (NOLOAD) :
	{
		. = ALIGN(4);
		__bss_sram3_start = .;
		*(.bss_sram3 .bss_sram3.*)
		. = ALIGN(4);
		__bss_sram3_end = .;
	} >CONT_SRAM1
	.noinit_sram3 (NOLOAD) :
	{
		. = ALIGN(4);
		__noinit_sram3_start = .;
		*(.noinit_sram3 .noinit_sram3.*)
		. = ALIGN(4);
		__noinit_sram3_end = .;
	} >CONT_SRAM1
	.heap_sram3 (NOLOAD) :
	{
		. = ALIGN(4);
		__heap_sram3_start = .;
		. = MAX(ABSOLUTE(.), ORIGIN(SRAM3) + LENGTH(SRAM3));
		__heap_sram3_end = .;
	} >CONT_SRAM1




	.data_backup :
	{
		. = ALIGN(4);
		__data_backup_load = LOADADDR(.data_backup);
		__data_backup_start = .;
		*(.data_backup .data_backup.*)
		. = ALIGN(4);
	} >BACKUP AT >FLASH

	__data_backup_end = .;

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


	/* CCM can only be accessed by D-Bus, do not place .fastcode here! */
	.fastdata :
	{
		. = ALIGN(4);
		__fastdata_load = LOADADDR(.fastdata);
		__fastdata_start = .;
		*(.fastdata .fastdata.*)
		. = ALIGN(4);
	} >CCM AT >FLASH

	.data_ccm :
	{
		__data_ccm_start = .;
		*(.data_ccm .data_ccm.*)
		. = ALIGN(4);
		__data_ccm_end = .;
	} >CCM AT >FLASH
	__fastdata_end = .;

	.bss_ccm (NOLOAD) :
	{
		. = ALIGN(4);
		__bss_ccm_start = .;
		*(.bss_ccm .bss_ccm.*)
		. = ALIGN(4);
		__bss_ccm_end = .;
	} >CCM
	.noinit_ccm (NOLOAD) :
	{
		. = ALIGN(4);
		__noinit_ccm_start = .;
		*(.noinit_ccm .noinit_ccm.*)
		. = ALIGN(4);
		__noinit_ccm_end = .;
	} >CCM
	.heap_ccm (NOLOAD) :
	{
		. = ALIGN(4);
		__heap_ccm_start = .;
		. = MAX(ABSOLUTE(.), ORIGIN(CCM) + LENGTH(CCM));
		__heap_ccm_end = .;
	} >CCM

    


	.rodata :
	{
		. = ALIGN(4);
		__table_zero_intern_start = .;
		LONG(__bss_start)
		LONG(__bss_end)
		LONG(__bss_sram2_start)
		LONG(__bss_sram2_end)
		LONG(__bss_sram3_start)
		LONG(__bss_sram3_end)
		LONG(__bss_backup_start)
		LONG(__bss_backup_end)
		LONG(__bss_ccm_start)
		LONG(__bss_ccm_end)
		__table_zero_intern_end = .;

		__table_copy_intern_start = .;
		LONG(__data_load)
		LONG(__data_start)
		LONG(__data_end)
		LONG(__data_sram2_load)
		LONG(__data_sram2_start)
		LONG(__data_sram2_end)
		LONG(__data_sram3_load)
		LONG(__data_sram3_start)
		LONG(__data_sram3_end)
		LONG(__data_backup_load)
		LONG(__data_backup_start)
		LONG(__data_backup_end)
		LONG(__fastdata_load)
		LONG(__fastdata_start)
		LONG(__fastdata_end)
		__table_copy_intern_end = .;

		__table_zero_extern_start = .;
        
		__table_zero_extern_end = .;
		__table_copy_extern_start = .;
        
		__table_copy_extern_end = .;

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
		__table_heap_start = .;
		LONG(0x001f)
		LONG(__heap_sram1_start)
		LONG(__heap_sram1_end)
		LONG(0x001f)
		LONG(__heap_sram2_start)
		LONG(__heap_sram2_end)
		LONG(0x001f)
		LONG(__heap_sram3_start)
		LONG(__heap_sram3_end)
		LONG(0x4009)
		LONG(__heap_backup_start)
		LONG(__heap_backup_end)
		LONG(0x2002)
		LONG(__heap_ccm_start)
		LONG(__heap_ccm_end)
        
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