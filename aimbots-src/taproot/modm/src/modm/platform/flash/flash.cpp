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

#include "flash.hpp"

static constexpr uint32_t FLASH_SR_ERR =
    FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR |
    FLASH_SR_INCERR | FLASH_SR_OPERR;
namespace modm::platform
{

bool
Flash::unlock()
{
	Flash::enable();
	if (FLASH->CR1 & FLASH_CR_LOCK)
	{
		FLASH->KEYR1 = 0x45670123;
		FLASH->KEYR1 = 0xCDEF89AB;
	}
	return not isLocked();
}

uint8_t
Flash::getPage(uintptr_t offset)
{
	const uintptr_t addr = OriginAddr + offset;
	return (addr - Bank1Addr) >> 17;
}

uint32_t
Flash::getOffset(uint8_t index)
{
	return (Bank1Addr - OriginAddr) + (1ul << 17) * index;
}

size_t
Flash::getSize([[maybe_unused]] uint8_t index)
{
	return (1ul	<< 17);
}

modm_ramcode uint32_t
Flash::erase(uint8_t index)
{
	FLASH->CCR1 = FLASH_SR_ERR;
	FLASH->CR1 = FLASH_CR_START | FLASH_CR_SER | FLASH_CR_PSIZE |
	             ((index << FLASH_CR_SNB_Pos) & FLASH_CR_SNB_Msk);
	while(isBusy()) ;
	const uint32_t err = FLASH->SR1 & FLASH_SR_ERR;
	FLASH->CR1 = 0;
	SCB_InvalidateDCache_by_Addr(getAddr(index), SectorSize);
	return err;
}

modm_ramcode uint32_t
Flash::program(uintptr_t addr, const MaxWordType &data)
{
	if (addr & (FlashWordSize - 1)) return FLASH_SR_PGSERR;
	FLASH->CCR1 = FLASH_SR_ERR;
	FLASH->CR1 = FLASH_CR_PG | FLASH_CR_PSIZE;
	__ISB(); __DSB();
	volatile uint32_t *dst = reinterpret_cast<volatile uint32_t *>(addr);
	dst[0] = data.data[0];
	__DSB();
	while (not (FLASH->SR1 & FLASH_SR_WBNE)) ;
	for (size_t ii = 1; ii < FlashWordWords; ++ii) {
		dst[ii] = data.data[ii];
	}
	__DSB();
	while (FLASH->SR1 & FLASH_SR_WBNE) ;
	while(isBusy()) ;
	const uint32_t err = FLASH->SR1 & FLASH_SR_ERR;
	FLASH->CR1 = 0;
	SCB_InvalidateDCache_by_Addr(reinterpret_cast<void *>(addr & ~uintptr_t(0x1f)), FlashWordSize);
	return err;
}

} // namespace modm::platform