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

#include <stdlib.h>
#include <cstdint>
#include <modm/debug/logger.hpp>
#include <modm/architecture/interface/assert.hpp>

using modm::AssertionHandler;
using modm::Abandonment;
using modm::AbandonmentBehavior;

extern AssertionHandler __assertion_table_start __asm("section$start$__DATA$modm_assertion");
extern AssertionHandler __assertion_table_end __asm("section$end$__DATA$modm_assertion");
// Since we use the default linker script on hosted, the above linker section are
// only included if something is put into these sections. Therefore we are placing
// an empty assertion handler here, which does not influence assertion handling.
Abandonment _modm_empty_assertion_handler(const modm::AssertionInfo &)
{ return Abandonment::DontCare; }
MODM_ASSERTION_HANDLER(_modm_empty_assertion_handler);
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
		abort();
	}
}

// Mingw64 :facepalm;
#if defined(__MINGW64__) && !defined(__clang__)
#define PRIuPTR "I64u"
#endif

modm_weak
void modm_abandon(const modm::AssertionInfo &info)
{
	MODM_LOG_ERROR.printf("Assertion '%s'", info.name);
	if (info.context != uintptr_t(-1)) { MODM_LOG_ERROR.printf(" @ %p (%" PRIuPTR ")", (void *)info.context, info.context); }
	MODM_LOG_ERROR.printf(" failed!\n  %s\nAbandoning...\n", info.description) << modm::flush;
}

}