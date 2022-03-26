/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)

#include "clock.hpp"

#include "modm/architecture/interface/assert.h"

namespace tap::arch::clock
{
static ClockStub *globalStubInstance = nullptr;

ClockStub::ClockStub()
{
    modm_assert(
        globalStubInstance == nullptr,
        "ClockStub",
        "multiple clock stubs defined at the same time");
    globalStubInstance = this;
}
ClockStub::~ClockStub() { globalStubInstance = nullptr; }

uint32_t getTimeMilliseconds()
{
    return globalStubInstance == nullptr ? 0 : globalStubInstance->time;
}

uint32_t getTimeMicroseconds()
{
    return globalStubInstance == nullptr ? 0 : 1000 * globalStubInstance->time;
}
}  // namespace tap::arch::clock

#endif
