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

#include "profiler.hpp"

#include "../algorithms/math_user_utils.hpp"

#include "clock.hpp"

#ifdef RUN_WITH_PROFILING

namespace tap
{
namespace arch
{
static int findIndex(
    modm::DynamicArray<modm::Pair<const char*, Profiler::MinMaxAvgStruct>>& times,
    const char* profile,
    bool shouldAdd)
{
    auto iter = times.begin();

    for (int i = 0; iter != times.end(); ++iter, ++i)
    {
        if ((*iter).getFirst() == profile || strcmp((*iter).getFirst(), profile) == 0)
        {
            return i;
        }
    }

    if (!shouldAdd)
    {
        return -1;
    }

    times.append({profile, {}});

    return times.getSize() - 1;
}

static Profiler::MinMaxAvgStruct find(
    modm::DynamicArray<modm::Pair<const char*, Profiler::MinMaxAvgStruct>>& times,
    const char* profile,
    bool shouldAdd)
{
    int index = findIndex(times, profile, shouldAdd);

    if (index != -1)
    {
        return times[index].getSecond();
    }

    return {};
}

void Profiler::push(const char* profile)
{
    profileStack.prepend(profile);
    profiles.append(profile);
    timeStack.prepend(tap::arch::clock::getTimeMicroseconds());
}
void Profiler::pop()
{
    const char* profile = profileStack.getFront();
    uint32_t ellapsedTime = tap::arch::clock::getTimeMicroseconds() - timeStack.getFront();
    timeStack.removeFront();
    profileStack.removeFront();

    MinMaxAvgStruct time = find(times, profile, true);

    if (time.min > ellapsedTime)
    {
        time.min = ellapsedTime;
    }
    else if (time.max < ellapsedTime)
    {
        time.max = ellapsedTime;
    }

    time.avg = algorithms::lowPassFilter(time.avg, ellapsedTime, 0.01f);
    time.totalValues += 1;

    times[findIndex(times, profile, false)].second = time;
}

uint64_t Profiler::getAvgTime(const char* profile) { return find(times, profile, false).avg; }
uint64_t Profiler::getMinTime(const char* profile) { return find(times, profile, false).min; }
uint64_t Profiler::getMaxTime(const char* profile) { return find(times, profile, false).max; }
void Profiler::reset(const char* profile) { times[findIndex(times, profile, false)].second = {}; }

}  // namespace arch

}  // namespace tap
#endif