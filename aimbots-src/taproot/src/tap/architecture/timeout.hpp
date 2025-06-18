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

#ifndef TAPROOT_TIMEOUT_HPP_
#define TAPROOT_TIMEOUT_HPP_

#include <cstdint>

#include "clock.hpp"

namespace tap
{
namespace arch
{
/**
 * A class for keeping track of a timer that expires. Template argument
 * expects a function pointer that returns a uin32_t representing some absolute
 * measure of time.
 *
 * Doesn't start until `restart()` is called
 */
template <uint32_t (*T)()>
class Timeout
{
    template <typename H>
    friend class PeriodicTimer;

private:
    bool isRunning;
    bool isExecuted;
    uint32_t expireTime;

public:
    static constexpr auto TimeFunc = T;

    Timeout()
    {
        stop();
        this->expireTime = 0;
    }

    explicit Timeout(uint32_t timeout) { restart(timeout); }

    /**
     * Set the timer to expire in `timeout` units of time.
     *
     * @param[in] timeout: the amount of time from when this function
     * is called that the timer should expire.
     */
    inline void restart(uint32_t timeout)
    {
        this->isRunning = true;
        this->isExecuted = false;
        this->expireTime = TimeFunc() + timeout;
    }

    /**
     * Stop the timer. If expired, the expiration flags are cleared.
     */
    inline void stop()
    {
        this->isRunning = false;
        this->isExecuted = false;
    }

    /**
     * @return `true` if the timer is stopped
     */
    inline bool isStopped() const { return !this->isRunning; }

    /**
     * @return `true` if the timer has expired (timeout has been reached) and is NOT
     * stopped.
     */
    inline bool isExpired() const { return this->isRunning && TimeFunc() >= this->expireTime; }

    /**
     * Returns `true` on the first call when timer has expired since restart. Use to
     * only catch the timeout expiration once.
     *
     * @return `true` the first time the timer has expired (timeout has been reached)
     * since last `restart()`
     */
    inline bool execute()
    {
        if (!isExecuted && isExpired())
        {
            isExecuted = true;
            return true;
        }

        return false;
    }
};

using MicroTimeout = Timeout<tap::arch::clock::getTimeMicroseconds>;
using MilliTimeout = Timeout<tap::arch::clock::getTimeMilliseconds>;
}  // namespace arch
}  // namespace tap

#endif  // TAPROOT_TIMEOUT_HPP_
