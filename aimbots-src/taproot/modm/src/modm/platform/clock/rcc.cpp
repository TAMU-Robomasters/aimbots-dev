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

namespace modm::platform
{

// ----------------------------------------------------------------------------
bool
Rcc::enableInternalClockMHz48(uint32_t waitCycles)
{
	bool retval;
	RCC->CR |= RCC_CR_HSI48ON;
	while (not (retval = (RCC->CR & RCC_CR_HSI48RDY)) and --waitCycles)
		;
	return retval;
}
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
Rcc::enablePll1(PllSource source, const PllFactors& pllFactors, uint32_t waitCycles)
{
	// set the PLL1 source and DIVM1 prescaler
	uint32_t tmp = RCC->PLLCKSELR & ~(RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1);
	tmp |= uint32_t(source);
	tmp |= (uint32_t(pllFactors.pllM) << RCC_PLLCKSELR_DIVM1_Pos) & RCC_PLLCKSELR_DIVM1;
	RCC->PLLCKSELR = tmp;

	// Set Frequency Range
	tmp = RCC->PLLCFGR & ~(RCC_PLLCFGR_PLL1RGE | RCC_PLLCFGR_PLL1VCOSEL);
	tmp |= uint32_t(pllFactors.range) << RCC_PLLCFGR_PLL1RGE_Pos;
	if (pllFactors.range == PllInputRange::MHz1_2) tmp |= RCC_PLLCFGR_PLL1VCOSEL;
	RCC->PLLCFGR = tmp;

	// Using integer mode
	RCC->PLL1FRACR = pllFactors.pllFrac;
	// Set PLL Dividers
	RCC->PLL1DIVR =
			((uint32_t(pllFactors.pllR - 1u) << RCC_PLL1DIVR_R1_Pos) & RCC_PLL1DIVR_R1) |
			((uint32_t(pllFactors.pllQ - 1u) << RCC_PLL1DIVR_Q1_Pos) & RCC_PLL1DIVR_Q1) |
			((uint32_t(pllFactors.pllP - 1u) << RCC_PLL1DIVR_P1_Pos) & RCC_PLL1DIVR_P1) |
			((uint32_t(pllFactors.pllN - 1u) << RCC_PLL1DIVR_N1_Pos) & RCC_PLL1DIVR_N1);

	// enable pll
	RCC->CR |= RCC_CR_PLL1ON;

	while (not (tmp = (RCC->CR & RCC_CR_PLL1RDY)) and --waitCycles)
		;

	return tmp;
}

bool
Rcc::disablePll1(uint32_t waitCycles)
{
	RCC->CR &= ~RCC_CR_PLL1ON;
	while ((RCC->CR & RCC_CR_PLL1RDY) and --waitCycles)
		;
	return waitCycles > 0;
}

bool
Rcc::enablePll2(PllSource source, const PllFactors& pllFactors, uint32_t waitCycles)
{
	// set the PLL2 source and DIVM2 prescaler
	uint32_t tmp = RCC->PLLCKSELR & ~(RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM2);
	tmp |= uint32_t(source);
	tmp |= (uint32_t(pllFactors.pllM) << RCC_PLLCKSELR_DIVM2_Pos) & RCC_PLLCKSELR_DIVM2;
	RCC->PLLCKSELR = tmp;

	// Set Frequency Range
	tmp = RCC->PLLCFGR & ~(RCC_PLLCFGR_PLL2RGE | RCC_PLLCFGR_PLL2VCOSEL);
	tmp |= uint32_t(pllFactors.range) << RCC_PLLCFGR_PLL2RGE_Pos;
	if (pllFactors.range == PllInputRange::MHz1_2) tmp |= RCC_PLLCFGR_PLL2VCOSEL;
	RCC->PLLCFGR = tmp;

	// Using integer mode
	RCC->PLL2FRACR = pllFactors.pllFrac;
	// Set PLL Dividers
	RCC->PLL2DIVR =
			((uint32_t(pllFactors.pllR - 1u) << RCC_PLL2DIVR_R2_Pos) & RCC_PLL2DIVR_R2) |
			((uint32_t(pllFactors.pllQ - 1u) << RCC_PLL2DIVR_Q2_Pos) & RCC_PLL2DIVR_Q2) |
			((uint32_t(pllFactors.pllP - 1u) << RCC_PLL2DIVR_P2_Pos) & RCC_PLL2DIVR_P2) |
			((uint32_t(pllFactors.pllN - 1u) << RCC_PLL2DIVR_N2_Pos) & RCC_PLL2DIVR_N2);

	// enable pll
	RCC->CR |= RCC_CR_PLL2ON;

	while (not (tmp = (RCC->CR & RCC_CR_PLL2RDY)) and --waitCycles)
		;

	return tmp;
}

bool
Rcc::disablePll2(uint32_t waitCycles)
{
	RCC->CR &= ~RCC_CR_PLL2ON;
	while ((RCC->CR & RCC_CR_PLL2RDY) and --waitCycles)
		;
	return waitCycles > 0;
}

bool
Rcc::enablePll3(PllSource source, const PllFactors& pllFactors, uint32_t waitCycles)
{
	// set the PLL3 source and DIVM3 prescaler
	uint32_t tmp = RCC->PLLCKSELR & ~(RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM3);
	tmp |= uint32_t(source);
	tmp |= (uint32_t(pllFactors.pllM) << RCC_PLLCKSELR_DIVM3_Pos) & RCC_PLLCKSELR_DIVM3;
	RCC->PLLCKSELR = tmp;

	// Set Frequency Range
	tmp = RCC->PLLCFGR & ~(RCC_PLLCFGR_PLL3RGE | RCC_PLLCFGR_PLL3VCOSEL);
	tmp |= uint32_t(pllFactors.range) << RCC_PLLCFGR_PLL3RGE_Pos;
	if (pllFactors.range == PllInputRange::MHz1_2) tmp |= RCC_PLLCFGR_PLL3VCOSEL;
	RCC->PLLCFGR = tmp;

	// Using integer mode
	RCC->PLL3FRACR = pllFactors.pllFrac;
	// Set PLL Dividers
	RCC->PLL3DIVR =
			((uint32_t(pllFactors.pllR - 1u) << RCC_PLL3DIVR_R3_Pos) & RCC_PLL3DIVR_R3) |
			((uint32_t(pllFactors.pllQ - 1u) << RCC_PLL3DIVR_Q3_Pos) & RCC_PLL3DIVR_Q3) |
			((uint32_t(pllFactors.pllP - 1u) << RCC_PLL3DIVR_P3_Pos) & RCC_PLL3DIVR_P3) |
			((uint32_t(pllFactors.pllN - 1u) << RCC_PLL3DIVR_N3_Pos) & RCC_PLL3DIVR_N3);

	// enable pll
	RCC->CR |= RCC_CR_PLL3ON;

	while (not (tmp = (RCC->CR & RCC_CR_PLL3RDY)) and --waitCycles)
		;

	return tmp;
}

bool
Rcc::disablePll3(uint32_t waitCycles)
{
	RCC->CR &= ~RCC_CR_PLL3ON;
	while ((RCC->CR & RCC_CR_PLL3RDY) and --waitCycles)
		;
	return waitCycles > 0;
}

bool
Rcc::setVoltageScaling(VoltageScaling voltage, uint32_t waitCycles)
{
	auto waitCounter = waitCycles;
	// wait for power supply to be ready to set VOS setting
	while(!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY))
		if (--waitCounter == 0) return false;

	// set voltage scale
	PWR->D3CR = static_cast<uint32_t>(voltage);
	waitCounter = waitCycles;
	while (!(PWR->D3CR & PWR_D3CR_VOSRDY))
		if (--waitCycles == 0) return false;
	return true;
}

bool
Rcc::configurePowerSource(PowerSource source, uint32_t waitCycles)
{
	constexpr auto mask = PWR_CR3_LDOEN |
		PWR_CR3_BYPASS;

	PWR->CR3 = (PWR->CR3 & ~mask) | static_cast<uint32_t>(source);

	// wait for power supply to be ready to set VOS setting
	while(!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY))
		if (--waitCycles == 0) return false;

	return true;
}
bool
Rcc::enableSystemClock(SystemClockSource src, uint32_t waitCycles)
{
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | uint32_t(src);

	// Wait till the main PLL is used as system clock source
	src = SystemClockSource(uint32_t(src) << RCC_CFGR_SWS_Pos);
	while ((RCC->CFGR & RCC_CFGR_SWS) != uint32_t(src))
		if (not --waitCycles) return false;

	return true;
}

}