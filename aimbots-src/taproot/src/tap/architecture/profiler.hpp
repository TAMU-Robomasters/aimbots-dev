/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_PROFILER_HPP_
#define TAPROOT_PROFILER_HPP_

#include <unordered_map>

#include "modm/container.hpp"

#ifdef RUN_WITH_PROFILING
#define PROFILE(profiler, func, params) \
    do                                  \
    {                                   \
        int key = profiler.push(#func); \
        func params;                    \
        profiler.pop(key);              \
    } while (0);
#else
#define PROFILE(profiler, func, params) func params
#endif

namespace tap
{
class Drivers;
}

namespace tap::arch
{
/**
 * An object that stores information about the time it takes to run code. User can add an item to
 * the profiler by pushing a profile to the profiler, running their function, then popping the
 * profile id from the profiler after the function has finished.
 *
 * The `PROFILE` macro allows you to interact with the profiler without using the profiler's
 * functions directly. For example, the `PROFILE` can be used as follows:
 *
 * ```cpp
 * void foo()
 * {
 * }
 *
 * void bar()
 * {
 *      PROFILE(profiler, foo, ());
 * }
 *
 * void baz()
 * {
 *      PROFILE(profiler, bar, ());
 * }
 * ```
 *
 * In the example above, `profiler` is a pointer to a valid `Profiler` class. If you call `baz`, it
 * will add the profile "bar" and "foo" to the profiler.
 *
 * The profiler is limited in size to `MAX_PROFILED_ELEMENTS`. Once a element is registered with the
 * profiler, the profiler will keep track of the min, max, and rolling average time (in
 * microseconds) it takes to run the code associated with the profile. The profiled elements can
 * then be inspected using a debugger, or in the future can be accessed via the terminal serial (not
 * yet implemented).
 *
 * One known limitation is this profiler doesn't handle recursion well. For example, consider the
 * following:
 *
 * ```cpp
 * void foo(int i)
 * {
 *      if (i == 0) return;
 *      PROFILE(profiler, foo, (i - 1));
 * }
 * ```
 *
 * In this example, the `PROFILE` macro will overwrite the entry in the profiler in a way that makes
 * the profiler information not useful for the `foo` function, so it is recommended that you do not
 * use the `PROFILE` macro for a recursive call.
 */
class Profiler
{
public:
    /// Max number of profiles that the profiler can store information about.
    static constexpr std::size_t MAX_PROFILED_ELEMENTS = 128;
    /// Low pass alpha to be used when averaging time it takes for some code to run.
    static constexpr float AVG_LOW_PASS_ALPHA = 0.01f;

    /**
     * Stores profile information.
     */
    struct ProfilerData
    {
        /// Name of the profile.
        const char* name = nullptr;
        /// Min value, in microseconds, ever recorded by the profiler.
        uint32_t min = UINT32_MAX;
        /// Max value, in microseconds, ever recorded by the profiler.
        uint32_t max = 0;
        /// Average value, in microseconds, averaged using a low pass filter.
        float avg = 0;
        /**
         * Value used to measure a "dt" between pushing and popping the profile from the profiler.
         */
        uint32_t prevPushedTime = 0;

        ProfilerData() {}
        explicit ProfilerData(const char* name) : name(name) {}

        /// Resets the long term profile storage information.
        void reset()
        {
            min = UINT32_MAX;
            max = 0;
            avg = 0;
        }
    };

    Profiler(tap::Drivers* drivers);

    /**
     * "Push" a profile to the profiler. Can be a new profile that the profiler has never seen or a
     * profile that is already in the profiler. Starts the stopwatch timer associated with the
     * profile.
     *
     * @param[in] profile The name of the profile. Should be a unique `const char *` (i.e. string
     * comparison is not used, instead raw pointer comparison is used).
     * @return A "key" that then must be passed to the `pop` function to stop the stopwatch timer
     * and update the profiles associated `ProfilerData`.
     */
    std::size_t push(const char* profile);

    /**
     * "Pops" a profile data, to stop the stopwatch that is timing how long some code takes to run.
     *
     * @param[in] key The key that was returned by `push` associated with the profile that you would
     * like to stop timing.
     */
    void pop(std::size_t key);

    /// @return The data associated with some particular key.
    inline ProfilerData getData(std::size_t key)
    {
        if (key >= profiledElements.getSize())
        {
            return ProfilerData();
        }
        else
        {
            return profiledElements.get(key);
        }
    }

    /// Reset the ProfilerData associated with some particular key.
    inline void reset(std::size_t key)
    {
        if (key < profiledElements.getSize())
        {
            profiledElements[key].reset();
        }
    }

private:
    tap::Drivers* drivers;

    /**
     * Map element names (function names) to index in profiledElements. Don't directly store
     * ProfilerData's in this map to allow for easier accessability of the elements during
     * debugging. std::map on the embedded system is scuffed and doesn't allow for easy access to
     * internal elements.
     */
    std::unordered_map<const char*, std::size_t> elementNameToIndexMap;

    /**
     * Array of profiling data information
     */
    modm::BoundedDeque<ProfilerData, MAX_PROFILED_ELEMENTS> profiledElements;
};

}  // namespace tap::arch

#endif  // TAPROOT_PROFILER_HPP_
