/*
 * Copyright (c) 2024, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <modm/platform/core/atomic_lock_impl.hpp>

/* We are implementing the libary interface described here:
 * See https://gcc.gnu.org/wiki/Atomic/GCCMM/LIbrary
 *
 * This header file must be included with <atomic>!
 */

[[gnu::always_inline]] inline void
__modm_atomic_pre_barrier([[maybe_unused]] int memorder)
{
	// A compiler fence is enough
	__atomic_signal_fence(__ATOMIC_SEQ_CST);
}

[[gnu::always_inline]] inline void
__modm_atomic_post_barrier([[maybe_unused]] int memorder)
{
	// A compiler fence is enough
	__atomic_signal_fence(__ATOMIC_SEQ_CST);
}

// ============================= generic integers =============================
template<typename T>
[[gnu::always_inline]] inline T
__modm_atomic_load_t(const volatile void *ptr, int memorder)
{
	T value{};
	__modm_atomic_pre_barrier(memorder);
	{
		modm::atomic::Lock _;
		value = *reinterpret_cast<const volatile T*>(ptr);
	}
	__modm_atomic_post_barrier(memorder);
	return value;
}

template<typename T>
[[gnu::always_inline]] inline void
__modm_atomic_store_t(volatile void *ptr, T value, int memorder)
{
	__modm_atomic_pre_barrier(memorder);
	{
		modm::atomic::Lock _;
		*reinterpret_cast<volatile T*>(ptr) = value;
	}
	__modm_atomic_post_barrier(memorder);
}

template<typename T>
[[gnu::always_inline]] inline T
__modm_atomic_exchange_t(volatile void *ptr, T desired, int memorder)
{
	T previous{};
	__modm_atomic_pre_barrier(memorder);
	{
		modm::atomic::Lock _;
		previous = *reinterpret_cast<volatile T*>(ptr);
		*reinterpret_cast<volatile T*>(ptr) = desired;
	}
	__modm_atomic_post_barrier(memorder);
	return previous;
}

template<typename T>
[[gnu::always_inline]] inline bool
__modm_atomic_compare_exchange_t(volatile void *ptr, void *expected, T desired, bool weak,
						  int success_memorder, int failure_memorder)
{
	bool retval{false};
	__modm_atomic_pre_barrier(weak ? success_memorder : __ATOMIC_SEQ_CST);
	{
		modm::atomic::Lock _;
		const T current = *reinterpret_cast<volatile T*>(ptr);
		if (current == *reinterpret_cast<T*>(expected)) [[likely]]
		{
			*reinterpret_cast<volatile T*>(ptr) = desired;
			retval = true;
		}
		else *reinterpret_cast<T*>(expected) = current;
	}
	__modm_atomic_post_barrier(weak ? (retval ? success_memorder : failure_memorder) : __ATOMIC_SEQ_CST);
	return retval;
}

// ================================ lock free =================================
extern "C" [[gnu::always_inline]] inline bool
__atomic_is_lock_free (unsigned int object_size, const volatile void *ptr)
{
	// only lock free if size ≤ bus width and then also properly aligned
	if (object_size <= 4) [[likely]]
		return ((uintptr_t)ptr & (object_size - 1)) == 0;
	return false;
}


// ========================= atomics for 64 bit integers =========================
extern "C" [[gnu::always_inline]] inline uint64_t
__atomic_load_8(const volatile void *ptr, int memorder)
{
	return __modm_atomic_load_t<uint64_t>(ptr, memorder);
}


extern "C" [[gnu::always_inline]] inline void
__atomic_store_8(volatile void *ptr, uint64_t value, int memorder)
{
	__modm_atomic_store_t<uint64_t>(ptr, value, memorder);
}

extern "C" [[gnu::always_inline]] inline uint64_t
__atomic_exchange_8(volatile void *ptr, uint64_t desired, int memorder)
{
	return __modm_atomic_exchange_t<uint64_t>(ptr, desired, memorder);
}


extern "C" [[gnu::always_inline]] inline bool
__atomic_compare_exchange_8(volatile void *ptr, void *expected, uint64_t desired,
							bool weak, int success_memorder, int failure_memorder)
{
	return __modm_atomic_compare_exchange_t<uint64_t>(
			ptr, expected, desired, weak, success_memorder, failure_memorder);
}


extern "C" [[gnu::always_inline]] inline uint64_t
__atomic_fetch_and_8(volatile void *ptr, uint64_t value, int memorder)
{
	uint64_t previous{};
	__modm_atomic_pre_barrier(memorder);
	{
		modm::atomic::Lock _;
		previous = *reinterpret_cast<volatile uint64_t*>(ptr);
		*reinterpret_cast<volatile uint64_t*>(ptr) = (previous & value);
	}
	__modm_atomic_post_barrier(memorder);
	return previous;
}
extern "C" [[gnu::always_inline]] inline uint64_t
__atomic_fetch_or_8(volatile void *ptr, uint64_t value, int memorder)
{
	uint64_t previous{};
	__modm_atomic_pre_barrier(memorder);
	{
		modm::atomic::Lock _;
		previous = *reinterpret_cast<volatile uint64_t*>(ptr);
		*reinterpret_cast<volatile uint64_t*>(ptr) = (previous | value);
	}
	__modm_atomic_post_barrier(memorder);
	return previous;
}
extern "C" [[gnu::always_inline]] inline uint64_t
__atomic_fetch_xor_8(volatile void *ptr, uint64_t value, int memorder)
{
	uint64_t previous{};
	__modm_atomic_pre_barrier(memorder);
	{
		modm::atomic::Lock _;
		previous = *reinterpret_cast<volatile uint64_t*>(ptr);
		*reinterpret_cast<volatile uint64_t*>(ptr) = (previous ^ value);
	}
	__modm_atomic_post_barrier(memorder);
	return previous;
}
extern "C" [[gnu::always_inline]] inline uint64_t
__atomic_fetch_nand_8(volatile void *ptr, uint64_t value, int memorder)
{
	uint64_t previous{};
	__modm_atomic_pre_barrier(memorder);
	{
		modm::atomic::Lock _;
		previous = *reinterpret_cast<volatile uint64_t*>(ptr);
		*reinterpret_cast<volatile uint64_t*>(ptr) = ~(previous & value);
	}
	__modm_atomic_post_barrier(memorder);
	return previous;
}

