/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009-2012, Fabian Greif
 * Copyright (c) 2011, Georgi Grinshpun
 * Copyright (c) 2012, 2016, Sascha Schade
 * Copyright (c) 2012, 2014-2019, 2021, Niklas Hauser
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2018, 2021, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "rcc.hpp"

// CMSIS Core compliance
constinit uint32_t modm_fastdata SystemCoreClock(modm::platform::Rcc::BootFrequency);
modm_weak void SystemCoreClockUpdate() { /* Nothing to update */ }

namespace modm::platform
{
constinit uint16_t modm_fastdata delay_fcpu_MHz(computeDelayMhz(Rcc::BootFrequency));
constinit uint16_t modm_fastdata delay_ns_per_loop(computeDelayNsPerLoop(Rcc::BootFrequency));

// ----------------------------------------------------------------------------
bool
Rcc::enableInternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->CR |= RCC_CR_HSION;
	while (not (retval = (RCC->CR & RCC_CR_HSIRDY)) and --waitCycles)
		;
	return retval;
}

bool
Rcc::enableExternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
	while (not (retval = (RCC->CR & RCC_CR_HSERDY)) and --waitCycles)
		;
	return retval;
}

bool
Rcc::enableExternalCrystal(uint32_t waitCycles)
{
	bool retval;
	RCC->CR = (RCC->CR & ~RCC_CR_HSEBYP) | RCC_CR_HSEON;
	while (not (retval = (RCC->CR & RCC_CR_HSERDY)) and --waitCycles)
		;
	return retval;
}

bool
Rcc::enableLowSpeedInternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->CSR |= RCC_CSR_LSION;
	while (not (retval = (RCC->CSR & RCC_CSR_LSIRDY)) and --waitCycles)
		;
	return retval;
}

bool
Rcc::enableLowSpeedExternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->BDCR |= RCC_BDCR_LSEBYP | RCC_BDCR_LSEON;
	while (not (retval = (RCC->BDCR & RCC_BDCR_LSERDY)) and --waitCycles)
		;
	return retval;
}

bool
Rcc::enableLowSpeedExternalCrystal(uint32_t waitCycles)
{
	bool retval;
	RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_LSEBYP) | RCC_BDCR_LSEON;
	while (not (retval = (RCC->BDCR & RCC_BDCR_LSERDY)) and --waitCycles)
		;
	return retval;
}

bool
Rcc::enablePll(PllSource source, const PllFactors& pllFactors, uint32_t waitCycles)
{
	// Read reserved values and clear all other values
	uint32_t tmp = RCC->PLLCFGR & ~(RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM
			| RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP);

	// PLLSRC source for pll and for plli2s
	tmp |= static_cast<uint32_t>(source);

	// PLLM (0) = factor is user defined VCO input frequency must be configured to 2MHz
	tmp |= ((uint32_t) pllFactors.pllM) & RCC_PLLCFGR_PLLM;

	// PLLN (6) = factor is user defined
	tmp |= (((uint32_t) pllFactors.pllN) << RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN;

	// PLLP (16) divider for CPU frequency; (00: PLLP = 2, 01: PLLP = 4, etc.)
	tmp |= (((uint32_t) (pllFactors.pllP / 2) - 1) << RCC_PLLCFGR_PLLP_Pos) & RCC_PLLCFGR_PLLP;

	// PLLQ (24) divider for USB frequency; (0-15)
	if (pllFactors.pllQ != 0xff) {
		tmp &= ~RCC_PLLCFGR_PLLQ;
		tmp |= (((uint32_t) pllFactors.pllQ) << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ;
	}

	RCC->PLLCFGR = tmp;

	// enable pll
	RCC->CR |= RCC_CR_PLLON;

	while (not (tmp = (RCC->CR & RCC_CR_PLLRDY)) and --waitCycles)
		;

	return tmp;
}

bool
Rcc::enableOverdriveMode(uint32_t waitCycles)
{
	PWR->CR |= PWR_CR_ODEN;
	auto waitCounter = waitCycles;
	while (!(PWR->CSR & PWR_CSR_ODRDY))
		if (--waitCounter == 0) return false;

	PWR->CR |= PWR_CR_ODSWEN;
	while (!(PWR->CSR & PWR_CSR_ODSWRDY))
		if (--waitCycles == 0) return false;

	return true;
}
// ----------------------------------------------------------------------------
bool
Rcc::enableSystemClock(SystemClockSource src, uint32_t waitCycles)
{
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | uint32_t(src);

	// Wait till the main PLL is used as system clock source
	src = SystemClockSource(uint32_t(src) << 2);
	while ((RCC->CFGR & RCC_CFGR_SWS) != uint32_t(src))
		if (not --waitCycles) return false;

	return true;
}


}