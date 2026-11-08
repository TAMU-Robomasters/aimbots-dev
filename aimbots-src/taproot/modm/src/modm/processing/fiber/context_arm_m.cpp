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

/* Stack layout (growing downwards):
 *
 * Permanent Storage:
 * Fiber Function
 * Fiber Function Argument
 *
 * Temporary Prepare:
 * Entry Function
 *
 * Register file: r4-r11 must be preserved across subroutine calls.
 *
 * LR
 * r11
 * r10
 * r9
 * r8
 * r7
 * r6
 * r5
 * r4
 * s31
 * s30
 * s29
 * s28
 * s27
 * s26
 * s25
 * s24
 * s23
 * s22
 * s21
 * s20
 * s19
 * s18
 * s17
 * s16
 *
 * From the PCSAA:
 * Registers s16-s31 (d8-d15, q4-q7) must be preserved across subroutine calls;
 * registers s0-s15 (d0-d7, q0- q3) do not need to be preserved (and can be used
 * for passing arguments or returning results in standard procedure-call
 * variants). Registers d16-d31 (q8-q15), if present, do not need to be
 * preserved.
 */

namespace
{

constexpr size_t StackWordsReset = 1;
constexpr size_t StackWordsStorage = 2;
constexpr size_t StackWordsRegisters = 25;
constexpr size_t StackWordsAll = StackWordsStorage + StackWordsRegisters;
constexpr size_t StackSizeWord = sizeof(uintptr_t);
constexpr uintptr_t StackWatermark = 0xf00d'cafe;

void modm_naked
modm_context_entry()
{
	asm volatile
	(
		"ldm sp, {r0, pc}	\n\t"	// Load data pointer and jump to closure
	);
}

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

#define MODM_PUSH_CONTEXT() \
		"push {r4-r11, lr}	\n\t" \
		"vpush {d8-d15}		\n\t"

#define MODM_POP_CONTEXT() \
		"vpop {d8-d15}		\n\t" \
		"pop {r4-r11, pc}	\n\t"

uintptr_t modm_naked
modm_context_start(modm_context_t*)
{
	asm volatile
	(
		MODM_PUSH_CONTEXT()

		"mrs r1, control	\n\t"
		"orr r1, r1, #2		\n\t"	// Set SPSEL
		"msr control, r1	\n\t"
		"isb				\t\n\t"

		"ldr sp, [r0]		\n\t"	// Set PSP to ctx->sp

		MODM_POP_CONTEXT()
	);
}

void modm_naked
modm_context_jump(modm_context_t*, modm_context_t*)
{
	asm volatile
	(
		MODM_PUSH_CONTEXT()

		"ldr r3, [r0, #4]		\n\t"	// Load from->bottom
		"str sp, [r0]			\n\t"	// Store the SP in from->sp

		"cmp sp, r3				\n\t"	// Compare SP to from->bottom
		"bls 1f					\n\t"	// If SP <= bottom, stack overflow

		"ldr r3, [r3]			\n\t"	// Load stack bottom
		"ldr r2, =%0			\n\t"	// Load StackWatermark value
		"cmp r2, r3				\n\t"	// Check if stack watermark is still at the bottom
		"bne 1f					\n\t"	// If not, stack overflow

		"ldr sp, [r1]			\n\t"	// Restore SP from to->sp

		MODM_POP_CONTEXT()

	"1:  b modm_context_end	\n\t"
		:: "i" (StackWatermark)
	);
}

void modm_naked
modm_context_end(uintptr_t)
{
	asm volatile
	(
		"mrs r1, control	\n\t"
		"bic r1, r1, #2		\n\t"	// Unset SPSEL
		"msr control, r1	\n\t"
		"isb				\n\t"

		MODM_POP_CONTEXT()
	);
}

