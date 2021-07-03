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

#ifndef PWM_HPP_
#define PWM_HPP_

#include <cstdint>

#include "aruwlib/util_macros.hpp"

namespace aruwlib
{
namespace gpio
{
/**
 * PWM input pins are pins S, T, U, and V (board pins PAO, PA1, PA2, PA3)
 * To write a PWM frequency to a pin call `write` and pass the function a
 * value (W - Z) from the analog outPin enum and a PWM duty from 0.0-1.0
 * (where 1 is all HIGH and 0 is all LOW). To set the duty for all pins
 * call the `writeAll` function with only the duty.
 */
class Pwm
{
public:
    Pwm() = default;
    DISALLOW_COPY_AND_ASSIGN(Pwm)
    mockable ~Pwm() = default;

    /**
     * PWM pins whose name corresponds to the names defined on the
     * RoboMaster type A board.
     */
    enum Pin
    {
        W = 1,
        X,
        Y,
        Z
    };

    mockable void init();

    /**
     * Sets all Timer channels to the same duty.
     *
     * @param[in] duty the duty cycle to be set. If the duty is outside of the range
     *      of [0, 1] the duty is limited to within the range.
     */
    mockable void writeAll(float duty);

    /**
     * Sets the PWM duty for a specified pin.
     *
     * @param [in] duty the duty cycle to be set. If the duty is outside of the range
     *      of [0, 1] the duty is limited to within the range.
     * @param[in] pin the PWM pin to be set.
     */
    mockable void write(float duty, Pwm::Pin pin);
};  // class Pwm

}  // namespace gpio

}  // namespace aruwlib

#endif  // PWM_HPP_
