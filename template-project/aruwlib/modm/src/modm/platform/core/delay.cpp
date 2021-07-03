/*
 * Copyright (c) 2015-2016, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "../device.hpp"
#include "hardware_init.hpp"
#include "delay_impl.hpp"

namespace modm
{

void modm_fastcode
delay_us(uint32_t us)
{
	modm_assert_continue_fail_debug(us <= 10'000'000ul,
		"delay.us", "modm::delay(us) can only delay a maximum of ~10 seconds!");
	if (us == 0) return;    // 1 cycle, or 2 when taken

	uint32_t start = DWT->CYCCNT;
	// prefer this for cores with fast hardware multiplication
	int32_t delay = int32_t(platform::delay_fcpu_MHz) * us - 25;

	while (int32_t(DWT->CYCCNT - start) < delay)
		;
}

}

void
modm_dwt_enable(void)
{
	// Enable Tracing Debug Unit
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	// Enable CPU cycle counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

MODM_HARDWARE_INIT_ORDER(modm_dwt_enable, 100);
