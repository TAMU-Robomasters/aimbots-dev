/*
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2013-2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "../device.hpp"
#include <modm/platform/core/hardware_init.hpp>

void
modm_gpio_enable(void)
{
	// Enable I/O compensation cell
	SYSCFG->CMPCR = SYSCFG_CMPCR_CMP_PD;
	// Enable GPIO clock
	RCC->AHB1ENR  |=
		RCC_AHB1ENR_GPIOAEN |
		RCC_AHB1ENR_GPIOBEN |
		RCC_AHB1ENR_GPIOCEN |
		RCC_AHB1ENR_GPIODEN |
		RCC_AHB1ENR_GPIOEEN |
		RCC_AHB1ENR_GPIOFEN |
		RCC_AHB1ENR_GPIOGEN |
		RCC_AHB1ENR_GPIOHEN |
		RCC_AHB1ENR_GPIOIEN;
	// Reset GPIO peripheral
	RCC->AHB1RSTR |=
		RCC_AHB1RSTR_GPIOARST |
		RCC_AHB1RSTR_GPIOBRST |
		RCC_AHB1RSTR_GPIOCRST |
		RCC_AHB1RSTR_GPIODRST |
		RCC_AHB1RSTR_GPIOERST |
		RCC_AHB1RSTR_GPIOFRST |
		RCC_AHB1RSTR_GPIOGRST |
		RCC_AHB1RSTR_GPIOHRST |
		RCC_AHB1RSTR_GPIOIRST;
	RCC->AHB1RSTR &= ~(
		RCC_AHB1RSTR_GPIOARST |
		RCC_AHB1RSTR_GPIOBRST |
		RCC_AHB1RSTR_GPIOCRST |
		RCC_AHB1RSTR_GPIODRST |
		RCC_AHB1RSTR_GPIOERST |
		RCC_AHB1RSTR_GPIOFRST |
		RCC_AHB1RSTR_GPIOGRST |
		RCC_AHB1RSTR_GPIOHRST |
		RCC_AHB1RSTR_GPIOIRST);
}

MODM_HARDWARE_INIT_ORDER(modm_gpio_enable, 80);