/*
 * Copyright (c) 2020, 2024, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm_atomic.hpp>

/* We are implementing the libary interface described here:
 * See https://gcc.gnu.org/wiki/Atomic/GCCMM/LIbrary
 *
 * We ignore the memory order, since the runtime switching takes longer than
 * the DMB instruction.
 *
 * These functions cannot be inlined, since the compiler builtins are named the
 * same. Terrible design really.
 */

// ============================ atomics for bools ============================
extern "C" bool
__atomic_test_and_set(volatile void *ptr, int /*memorder*/)
{
	bool value{};
	__modm_atomic_pre_barrier(__ATOMIC_SEQ_CST);
	{
		modm::atomic::Lock _;
		value = *reinterpret_cast<volatile bool*>(ptr);
		*reinterpret_cast<volatile bool*>(ptr) = true;
	}
	__modm_atomic_post_barrier(__ATOMIC_SEQ_CST);
	return value;
}

// ============================ atomics for arrays ============================
extern "C" void
__atomic_load(unsigned int size, const volatile void *src, void *dest, int /*memorder*/)
{
	__modm_atomic_pre_barrier(__ATOMIC_SEQ_CST);
	{
		modm::atomic::Lock _;
		__builtin_memcpy(dest, (const void*)src, size);
	}
	__modm_atomic_post_barrier(__ATOMIC_SEQ_CST);
}

extern "C" void
__atomic_store(unsigned int size, volatile void *dest, void *src, int /*memorder*/)
{
	__modm_atomic_pre_barrier(__ATOMIC_SEQ_CST);
	{
		modm::atomic::Lock _;
		__builtin_memcpy((void*)dest, src, size);
	}
	__modm_atomic_post_barrier(__ATOMIC_SEQ_CST);
}

extern "C" void
__atomic_exchange(unsigned int size, volatile void *ptr, void *val, void *ret, int /*memorder*/)
{
	__modm_atomic_pre_barrier(__ATOMIC_SEQ_CST);
	{
		modm::atomic::Lock _;
		__builtin_memcpy(ret, (void*)ptr, size);
		__builtin_memcpy((void*)ptr, val, size);
	}
	__modm_atomic_post_barrier(__ATOMIC_SEQ_CST);
}

extern "C" bool
__atomic_compare_exchange(unsigned int len, volatile void *ptr, void *expected, void *desired,
						  int /*success_memorder*/, int /*failure_memorder*/)
{
	bool retval{false};
	__modm_atomic_pre_barrier(__ATOMIC_SEQ_CST);
	{
		modm::atomic::Lock _;
		if (__builtin_memcmp((void*)ptr, expected, len) == 0) [[likely]]
		{
			__builtin_memcpy((void*)ptr, desired, len);
			retval = true;
		}
		else __builtin_memcpy(expected, (void*)ptr, len);
	}
	__modm_atomic_post_barrier(__ATOMIC_SEQ_CST);
	return retval;
}

// ========================= atomics for 64 bit integers =========================
// These functions cannot be inlined since the compiler refuses to find these
// functions even if they are declared right at the call site. Unclear why.
extern "C" uint64_t
__atomic_fetch_add_8(volatile void *ptr, uint64_t value, int /*memorder*/)
{
	uint64_t previous{};
	__modm_atomic_pre_barrier(__ATOMIC_SEQ_CST);
	{
		modm::atomic::Lock _;
		previous = *reinterpret_cast<volatile uint64_t*>(ptr);
		*reinterpret_cast<volatile uint64_t*>(ptr) = (previous + value);
	}
	__modm_atomic_post_barrier(__ATOMIC_SEQ_CST);
	return previous;
}
extern "C" uint64_t
__atomic_fetch_sub_8(volatile void *ptr, uint64_t value, int /*memorder*/)
{
	uint64_t previous{};
	__modm_atomic_pre_barrier(__ATOMIC_SEQ_CST);
	{
		modm::atomic::Lock _;
		previous = *reinterpret_cast<volatile uint64_t*>(ptr);
		*reinterpret_cast<volatile uint64_t*>(ptr) = (previous - value);
	}
	__modm_atomic_post_barrier(__ATOMIC_SEQ_CST);
	return previous;
}

