/*
 * Copyright (c) 2016-2020, Niklas Hauser
 * Copyright (c) 2017, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/platform/device.hpp>
#include <modm/architecture/interface/assert.hpp>

using modm::AssertionHandler;
using modm::Abandonment;
using modm::AbandonmentBehavior;

extern AssertionHandler __assertion_table_start;
extern AssertionHandler __assertion_table_end;
extern "C"
{

void
modm_assert_report(_modm_assertion_info *cinfo)
{
	auto info = reinterpret_cast<modm::AssertionInfo *>(cinfo);
	uint8_t behavior(uint8_t(info->behavior));

	for (const AssertionHandler *handler = &__assertion_table_start;
		 handler < &__assertion_table_end; handler++)
	{
		behavior |= (uint8_t)(*handler)(*info);
	}

	info->behavior = AbandonmentBehavior(behavior);
	behavior &= ~uint8_t(Abandonment::Debug);
	if ((behavior == uint8_t(Abandonment::DontCare)) or
		(behavior & uint8_t(Abandonment::Fail)))
	{
		modm_abandon(*info);
		NVIC_SystemReset();
	}
}

modm_weak
void modm_abandon(const modm::AssertionInfo &info)
{
	(void)info;
}

}