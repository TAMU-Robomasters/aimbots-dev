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
	AbandonmentBehavior behavior(info->behavior);

	for (const AssertionHandler *handler = &__assertion_table_start;
		 handler < &__assertion_table_end; handler++)
	{
		behavior |= (*handler)(*info);
	}

	info->behavior = behavior;
	behavior.reset(Abandonment::Debug);
	if ((behavior == Abandonment::DontCare) or
		(behavior & Abandonment::Fail))
	{
		modm_abandon(*info);
		NVIC_SystemReset();
	}
}

// Mingw64 :facepalm;
#if defined(__MINGW64__) && !defined(__clang__)
#define PRIuPTR "I64u"
#endif

modm_weak
void modm_abandon(const modm::AssertionInfo &info)
{
	(void)info;
}

}