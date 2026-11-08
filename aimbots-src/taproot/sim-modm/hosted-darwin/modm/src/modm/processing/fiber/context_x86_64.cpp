/*
 * Copyright (c) 2020, Erik Henriksson
 * Copyright (c) 2021, 2023, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "context.h"
#include <modm/architecture/detect.hpp>

/* Stack layout (growing downwards):
 *
 * Permanent Storage:
 * Fiber Function
 * Fiber Function Argument
 *
 * Temporary Prepare:
 * Entry Function
 *
 * Register file:
 *
 * fc_mxcsr | fc_x87_cw
 * SEE registers XMM6-XMM15 (for windows)
 * rbp
 * rbx
 * rsi (for windows)
 * rdi (for windows)
 * r15
 * r14
 * r13
 * r12
 */

namespace
{

constexpr size_t StackWordsReset = 1;
constexpr size_t StackWordsStorage = 2;
constexpr size_t StackWordsRegisters = 30;
constexpr size_t StackWordsAll = StackWordsStorage + StackWordsRegisters;
constexpr size_t StackSizeWord = sizeof(uintptr_t);
constexpr uintptr_t StackWatermark = 0xc0ffee'f00d'facade;

}

uintptr_t modm_context_jump_entry(modm_context_t *from, modm_context_t *to);
void modm_context_jump_return(uintptr_t, modm_context_t*) asm("modm_context_jump_return");

void modm_naked
modm_context_entry()
{
	asm volatile
	(
		"mov  (%rsp), %rdi	\n\t" // Load argument pointer
		"mov 8(%rsp), %rsi	\n\t" // Load function pointer
		"jmp  *%rsi			\n\t" // Jump into function
	);
}

void
modm_context_init(modm_context_t *ctx,
				  uintptr_t *bottom, uintptr_t *top,
				  uintptr_t fn, uintptr_t fn_arg)
{
	ctx->bottom = bottom;
	ctx->top = top;

	ctx->sp = top;
	*--ctx->sp = fn;
	*--ctx->sp = fn_arg;
}

void
modm_context_reset(modm_context_t *ctx)
{
	*ctx->bottom = StackWatermark;

	ctx->sp = ctx->top - StackWordsStorage;
	*--ctx->sp = (uintptr_t) modm_context_entry;
	ctx->sp -= StackWordsRegisters - StackWordsReset;
	// initialize stack with the right flags
	asm volatile
	(
		"stmxcsr 0xa0(%0)	\n\t"
		"fnstcw  0xa4(%0)	\n\t"
		:: "r" (ctx->sp)
	);
}

void
modm_context_stack_watermark(modm_context_t *ctx)
{
	// clear the register file on the stack
	for (auto *word = ctx->top - StackWordsAll;
		 word < ctx->top - StackWordsStorage - StackWordsReset; word++)
		*word = 0;

	// then color the whole stack *below* the register file
	for (auto *word = ctx->bottom; word < ctx->top - StackWordsAll; word++)
		*word = StackWatermark;
}

size_t
modm_context_stack_usage(const modm_context_t *ctx)
{
	for (auto *word = ctx->bottom; word < ctx->top; word++)
		if (StackWatermark != *word)
			return (ctx->top - word) * StackSizeWord;
	return 0;
}

size_t
modm_context_stack_size(const modm_context_t *ctx)
{
	return (ctx->top - ctx->bottom) * StackSizeWord;
}

static modm_context_t main_context;

uintptr_t
modm_context_start(modm_context_t *to)
{
	return modm_context_jump_entry(&main_context, to);
}

void
modm_context_end(uintptr_t retval)
{
	modm_context_jump_return(retval, &main_context);
	__builtin_unreachable();
}

void
modm_context_jump(modm_context_t *from, modm_context_t *to)
{
	register uintptr_t* sp asm("rsp");
	if ((sp - StackWordsRegisters) < from->bottom or *from->bottom != StackWatermark)
		modm_context_end((uintptr_t) from);
	modm_context_jump_entry(from, to);
}

/*
The assembly code below is adapted from the Boost Context library to work
for Windows, Linux and macOS.
See https://github.com/boostorg/context/tree/develop/src/asm
- Windows: jump_x86_64_ms_pe_clang_gas.S
- Linux: jump_x86_64_sysv_elf_gas.S
- macOS: jump_x86_64_sysv_macho_gas.S

			Copyright Oliver Kowalke 2009.
   Distributed under the Boost Software License, Version 1.0.
	  (See accompanying file LICENSE_1_0.txt or copy at
			http://www.boost.org/LICENSE_1_0.txt)
*/

uintptr_t modm_naked
modm_context_jump_entry(modm_context_t*, modm_context_t*)
{
	asm volatile
	(
		"leaq -0xe8(%rsp), %rsp		\n\t"	// move stack pointer down

		"stmxcsr 0xa0(%rsp)			\n\t"	// save MMX control- and status-word
		"fnstcw  0xa4(%rsp)			\n\t"	// save x87 control-word

		"movq %r12, 0xa8(%rsp)		\n\t"	// save R12
		"movq %r13, 0xb0(%rsp)		\n\t"	// save R13
		"movq %r14, 0xb8(%rsp)		\n\t"	// save R14
		"movq %r15, 0xc0(%rsp)		\n\t"	// save R15
		"movq %rbx, 0xd8(%rsp)		\n\t"	// save RBX
		"movq %rbp, 0xe0(%rsp)		\n\t"	// save RBP

		"movq %rsp, (%rdi)			\n\t"	// Store the SP in "from"
	"1:  movq (%rsi), %rsp			\n\t"	// Restore SP from "to"

		"ldmxcsr 0xa0(%rsp)			\n\t"	// restore MMX control- and status-word
		"fldcw   0xa4(%rsp)			\n\t"	// restore x87 control-word

		"movq 0xa8(%rsp),  %r12		\n\t"	// restore R12
		"movq 0xb0(%rsp),  %r13		\n\t"	// restore R13
		"movq 0xb8(%rsp),  %r14		\n\t"	// restore R14
		"movq 0xc0(%rsp),  %r15		\n\t"	// restore R15
		"movq 0xd8(%rsp), %rbx		\n\t"	// restore RBX
		"movq 0xe0(%rsp), %rbp		\n\t"	// restore RBP

		"leaq 0xe8(%rsp), %rsp		\n\t"	// move stack pointer up

		"ret						\n\t"	// Perform the jump back

	"modm_context_jump_return:		\n\t"
		"mov %rdi, %rax				\n\t"	// Move first argument to return register
		"jmp 1b						\n\t"
	);
}