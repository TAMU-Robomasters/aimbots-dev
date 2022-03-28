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

#ifndef TAPROOT_PWM_HPP_
#define TAPROOT_PWM_HPP_

#include <cstdint>

#include "tap/util_macros.hpp"

namespace tap
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

    static constexpr uint32_t DEFAULT_TIMER8_FREQUENCY = 2000;

    static constexpr uint32_t DEFAULT_TIMER12_FREQUENCY = 2000;

    static constexpr uint32_t DEFAULT_TIMER3_FREQUENCY = 2000;

    Pwm() = default;
    DISALLOW_COPY_AND_ASSIGN(Pwm)
    mockable ~Pwm() = default;

    enum Pin
    {
        W,
        X,
        Y,
        Z,
        Buzzer,
        ImuHeater,
    };

    enum Timer
    {
        TIMER8,
        TIMER12,
        TIMER3,
    };

    mockable void init();

    /**
     * Sets all configured timer channels to 0% duty cycle.
     */
    mockable void writeAllZeros();

    /**
     * Sets the PWM duty for a specified pin.
     *
     * @param [in] duty the duty cycle to be set. If the duty is outside of the range
     *      of [0, 1] the duty is limited to within the range.
     * @param[in] pin the PWM pin to be set.
     */
    mockable void write(float duty, Pwm::Pin pin);

    /**
     * Set the frequency of the timer, in Hz. Does nothing if frequency == 0
     */
    mockable void setTimerFrequency(Timer timer, uint32_t frequency);

    mockable void pause(Timer timer);

    mockable void start(Timer timer);

private:
    static constexpr int BUZZER_CHANNEL = 1;
    static constexpr int HEATER_CHANNEL = 2;

    enum Ch
    {
        Ch1 = 1,
        Ch2 = 2,
        Ch3 = 3,
        Ch4 = 4,
    };

    /**
     * Overflow as calculated by the modm Timer8 object in its getPeriod function.
     * This is what the Auto Reload Register is set to and the pwm duty is scaled to
     * a value between 0 and this value.
     */
    uint16_t timer8CalculatedOverflow;
    /**
     * Overflow as calculated by the modm Timer12 object in its getPeriod function.
     * This is what the Auto Reload Register is set to and the pwm duty is scaled to
     * a value between 0 and this value.
     */
    uint16_t timer12CalculatedOverflow;
    /**
     * Overflow as calculated by the modm Timer3 object in its getPeriod function.
     * This is what the Auto Reload Register is set to and the pwm duty is scaled to
     * a value between 0 and this value.
     */
    uint16_t timer3CalculatedOverflow;
};  // class Pwm

}  // namespace gpio

}  // namespace tap

#endif  // TAPROOT_PWM_HPP_
