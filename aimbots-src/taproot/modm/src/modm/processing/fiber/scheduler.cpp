/*
 * Copyright (c) 2023, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "scheduler.hpp"

/// @cond
namespace modm::this_fiber
{

void
yield()
{
	modm::fiber::Scheduler::instance().yield();
}

modm::fiber::id
get_id()
{
	return modm::fiber::Scheduler::instance().get_id();
}

} // namespace modm::this_fiber
/// @endcond
