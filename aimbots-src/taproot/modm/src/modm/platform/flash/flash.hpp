/*
 * Copyright (c) 2020, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once
#include "../device.hpp"
#include <modm/platform/clock/rcc.hpp>
#include <modm/architecture/interface/register.hpp>

namespace modm::platform
{

/// @ingroup modm_platform_flash
class Flash
{
public:
	static constexpr uintptr_t OriginAddr{ 0x8000000 };
	static constexpr size_t Size{ 0x100000 };
	static inline uint8_t *const Origin{(uint8_t*)OriginAddr};
	static constexpr uintptr_t Bank1Addr{ 0x8000000 };
	static constexpr uintptr_t Bank2Addr{ 0x8000000 };
	static constexpr size_t SectorSize{ 1ul << 17 };
	static constexpr uint8_t SectorsPerBank{ 8 };
	static constexpr size_t FlashWordWords{ FLASH_NB_32BITWORD_IN_FLASHWORD };
	static constexpr size_t FlashWordSize{ FlashWordWords * sizeof(uint32_t) };
	struct alignas(4) MaxWordType
	{
		uint32_t data[FlashWordWords];
	};
	static_assert(sizeof(MaxWordType) == FlashWordSize);
	static_assert(alignof(MaxWordType) == alignof(uint32_t));
public:
	inline static void
	enable()
	{
	}

	inline static void
	disable()
	{
	}

	static bool
	isLocked()
	{
		return FLASH->CR1 & FLASH_CR_LOCK;
	}

	static inline bool
	isBusy()
	{
		return FLASH->SR1 & (FLASH_SR_BSY | FLASH_SR_QW);
	}

	static bool
	unlock();

	static inline uint8_t
	getSector(uint8_t *ptr)
	{ return getPage(ptr - Flash::Origin); }

	static uint8_t
	getSector(uintptr_t offset)
	{ return getPage(offset); }

	static inline uint8_t
	getPage(uint8_t *ptr)
	{ return getPage(ptr - Flash::Origin); }

	static uint8_t
	getPage(uintptr_t offset);

	static inline uint8_t*
	getAddr(uint8_t sector)
	{
		return Origin + getOffset(sector);
	}

	static uint32_t
	getOffset(uint8_t sector);

	static size_t
	getSize(uint8_t sector);

	static uint32_t
	erase(uint8_t sector);
	static uint32_t
	program(uintptr_t addr, const MaxWordType &data);
};

} // namespace modm::platform