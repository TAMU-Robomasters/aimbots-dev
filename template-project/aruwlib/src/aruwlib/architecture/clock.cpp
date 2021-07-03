/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)

#include "clock.hpp"

namespace aruwlib
{
namespace arch
{
namespace clock
{
uint32_t currTimeMilliseconds = 0;

void setTime(uint32_t timeMilliseconds) { currTimeMilliseconds = timeMilliseconds; }

uint32_t getTimeMilliseconds() { return currTimeMilliseconds; }

uint32_t getTimeMicroseconds() { return currTimeMilliseconds * 1000; }
}  // namespace clock
}  // namespace arch
}  // namespace aruwlib

#endif
