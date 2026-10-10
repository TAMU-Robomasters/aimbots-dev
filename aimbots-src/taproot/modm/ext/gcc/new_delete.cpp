/*
 * Copyright (c) 2009-2010, Martin Rosekeit
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2011-2012, 2014, 2016, 2020, Niklas Hauser
 * Copyright (c) 2012, Sascha Schade
 * Copyright (c) 2013, Kevin Läufer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <new>
#include <stdlib.h> // for prototypes of malloc() and free()
#include <modm/architecture/interface/memory.hpp>
#include <modm/architecture/interface/assert.hpp>

extern "C" modm_weak
void* malloc_traits(std::size_t size, uint32_t)
{ return malloc(size); }
template<bool with_traits>
static inline void*
new_assert(size_t size, [[maybe_unused]] modm::MemoryTraits traits = modm::MemoryDefault)
{
	void *ptr;
	while(1)
	{
		if constexpr (with_traits) {
			ptr = malloc_traits(size, traits.value);
		} else {
			ptr = malloc(size);
		}
		if (ptr) break;
		/* See footnote 1) in https://en.cppreference.com/w/cpp/memory/new/operator_new
		 *
		 * In case of failure, the standard library implementation calls the
		 * function pointer returned by std::get_new_handler and repeats
		 * allocation attempts until new handler does not return or becomes a
		 * null pointer, at which time it throws std::bad_alloc.
		 */
		if (std::get_new_handler()) std::get_new_handler()();
		else break;
	}
	if (ptr == nullptr) {
		modm_assert(0, "new",
			"C++ new() operator failed to allocate!", size);
	}
	return ptr;
}

// ----------------------------------------------------------------------------
modm_weak
void* operator new  (std::size_t size) { return new_assert<false>(size); }
modm_weak
void* operator new[](std::size_t size) { return new_assert<false>(size); }

modm_weak
void* operator new  (std::size_t size, const std::nothrow_t&) noexcept { return malloc(size); }
modm_weak
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept { return malloc(size); }

modm_weak
void* operator new  (std::size_t size, std::align_val_t) { return new_assert<false>(size); }
modm_weak
void* operator new[](std::size_t size, std::align_val_t) { return new_assert<false>(size); }

modm_weak
void* operator new  (std::size_t size, modm::MemoryTraits traits) { return new_assert<true>(size, traits); }
modm_weak
void* operator new[](std::size_t size, modm::MemoryTraits traits) { return new_assert<true>(size, traits); }

modm_weak
void* operator new  (std::size_t size, modm::MemoryTraits traits, const std::nothrow_t&) noexcept { return malloc_traits(size, traits.value); }
modm_weak
void* operator new[](std::size_t size, modm::MemoryTraits traits, const std::nothrow_t&) noexcept { return malloc_traits(size, traits.value); }
// ----------------------------------------------------------------------------
extern "C" modm_weak
void operator_delete([[maybe_unused]] void* ptr)
{
	free(ptr);
}

modm_weak
void operator delete  (void* ptr) noexcept { operator_delete(ptr); }
modm_weak
void operator delete[](void* ptr) noexcept { operator_delete(ptr); }

modm_weak
void operator delete  (void* ptr, std::size_t) noexcept { operator_delete(ptr); }
modm_weak
void operator delete[](void* ptr, std::size_t) noexcept { operator_delete(ptr); }

modm_weak
void operator delete  (void* ptr, const std::nothrow_t&) noexcept { operator_delete(ptr); }
modm_weak
void operator delete[](void* ptr, const std::nothrow_t&) noexcept { operator_delete(ptr); }

modm_weak
void operator delete  (void* ptr, std::size_t, std::align_val_t) noexcept { operator_delete(ptr); }
modm_weak
void operator delete[](void* ptr, std::size_t, std::align_val_t) noexcept { operator_delete(ptr); }