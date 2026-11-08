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
	SYSCFG->CCCSR = SYSCFG_CCCSR_EN;
	// Enable GPIO clock
	RCC->AHB4ENR  |=
		RCC_AHB4ENR_GPIOAEN |
		RCC_AHB4ENR_GPIOBEN |
		RCC_AHB4ENR_GPIOCEN |
		RCC_AHB4ENR_GPIODEN |
		RCC_AHB4ENR_GPIOEEN |
		RCC_AHB4ENR_GPIOHEN;
	// Reset GPIO peripheral
	RCC->AHB4RSTR |=
		RCC_AHB4RSTR_GPIOARST |
		RCC_AHB4RSTR_GPIOBRST |
		RCC_AHB4RSTR_GPIOCRST |
		RCC_AHB4RSTR_GPIODRST |
		RCC_AHB4RSTR_GPIOERST |
		RCC_AHB4RSTR_GPIOHRST;
	RCC->AHB4RSTR &= ~(
		RCC_AHB4RSTR_GPIOARST |
		RCC_AHB4RSTR_GPIOBRST |
		RCC_AHB4RSTR_GPIOCRST |
		RCC_AHB4RSTR_GPIODRST |
		RCC_AHB4RSTR_GPIOERST |
		RCC_AHB4RSTR_GPIOHRST);
}

MODM_HARDWARE_INIT_ORDER(modm_gpio_enable, 80);