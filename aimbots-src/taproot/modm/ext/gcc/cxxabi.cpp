/*
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2012, Sascha Schade
 * Copyright (c) 2012-2014, 2020, 2024, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/architecture/interface/assert.hpp>

extern "C"
{

void *__dso_handle = &__dso_handle;
void __cxa_pure_virtual()
{ modm_assert(0, "virt.pure", "A pure virtual function was called!"); }
void __cxa_deleted_virtual()
{ modm_assert(0, "virt.del", "A deleted virtual function was called!"); }

}

#include <atomic>
#include <modm/processing/fiber.hpp>
#include <modm/platform/device.hpp>

// One-time construction API, see ARM IHI0041D section 3.2.3.
// The ARM C++ ABI mandates the guard to be 32-bit aligned, 32-bit values.
using guard_type = uint32_t;

enum
{
	UNINITIALIZED = 0,
	INITIALIZED = 1,
	INITIALIZING = 0x10,
};

// This function returns 1 only if the object needs to be initialized
extern "C" int
__cxa_guard_acquire(guard_type *guard)
{
	auto atomic_guard = std::atomic_ref(*guard);
	guard_type value = atomic_guard.load(std::memory_order_relaxed);
	do
	{
		if (value == INITIALIZED) return 0;
		if (value == INITIALIZING)
		{
			const bool is_in_irq = __get_IPSR();
			// We got called from inside an interrupt, but we cannot yield back
			modm_assert(not is_in_irq, "stat.rec",
					"Recursive initialization of a function static!", guard);
			// we're not in an interrupt, try to yield back to the initializing fiber
			modm::this_fiber::yield();
		}
		value = UNINITIALIZED;
	}
	while(not atomic_guard.compare_exchange_weak(value, INITIALIZING,
						std::memory_order_acquire, std::memory_order_relaxed));
	return 1;
}

// After this function the compiler expects `(guard & 1) == 1`!
extern "C" void
__cxa_guard_release(guard_type *guard)
{
	auto atomic_guard = std::atomic_ref(*guard);
	atomic_guard.store(INITIALIZED, std::memory_order_release);
}

// Called if the initialization terminates by throwing an exception.
// After this function the compiler expects `(guard & 3) == 0`!
extern "C" void
__cxa_guard_abort([[maybe_unused]] guard_type *guard)
{
	auto atomic_guard = std::atomic_ref(*guard);
	atomic_guard.store(UNINITIALIZED, std::memory_order_release);
}
