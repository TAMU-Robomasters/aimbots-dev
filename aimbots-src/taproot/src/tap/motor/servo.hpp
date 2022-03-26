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

#ifndef TAPROOT_SERVO_HPP_
#define TAPROOT_SERVO_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/communication/gpio/pwm.hpp"

namespace tap
{
class Drivers;
namespace motor
{
/**
 * This class wraps around the PWM class to provide utilities for controlling a servo.
 * In particular, this class limits some target PWM to a min and max PWM value and uses
 * ramping to control the speed of the servo.
 */
class Servo
{
public:
    /**
     * Initializes the PWM bounds and associates the Servo with some PWM pin.
     *
     * @note `maximumPwm` and `minimumPwm` are limited to between [0, 1]. Also if
     *      `maximumPwm` < `minimumPwm`, an error is thrown and [minPwm, maxPwm]
     *      is set to [0, 1].
     *
     * @param[in] drivers Instance to the drivers class you would like to use.
     * @param[in] pwmPin The pin to attach the Servo class with.
     * @param[in] maximumPwm The maximum allowable PWM output. This is limited between 0 and 1.
     * @param[in] minimumPwm The minimum allowable PWM output. This is limited between 0 and 1.
     * @param[in] pwmRampSpeed The speed in PWM percent per millisecond.
     */
    Servo(
        Drivers *drivers,
        tap::gpio::Pwm::Pin currpwmPinPort,
        float maximumPwm,
        float minimumPwm,
        float pwmRampSpeed);

    /**
     * Limits `pwmOutputRamp` to `minPwm` and `maxPwm`, then sets ramp output
     * to the limited value. Do not repeatedly call (i.e. only call in a `Command`'s
     * `initialize` function, for example).
     */
    void setTargetPwm(float PWM);

    /**
     * Updates the `pwmOutputRamp` object and then sets the output PWM to the updated
     * ramp value.
     */
    void updateSendPwmRamp();

    /**
     * @return The current PWM output to the servo.
     */
    float getPWM() const;

    /**
     * @return The minimum PWM output (as a duty cycle).
     */
    float getMinPWM() const;

    /**
     * @return The maximum PWM output (as a duty cycle).
     */
    float getMaxPWM() const;

    /**
     * @return `true` if the ramp has met the desired PWM value (set with `setTargetPwm`).
     *      Use this to estimate when a servo movement is complete.
     */
    bool isRampTargetMet() const;

private:
    Drivers *drivers;

    /// Used to change servo speed. See construtctor for detail.
    tap::algorithms::Ramp pwmOutputRamp;

    /// The max PWM the servo can handle.
    float maxPwm;

    /// The min PWM the servo can handle.
    float minPwm;

    /// Current PWM output.
    float currentPwm;

    /// Desired speed of the ramp in PWM / ms
    float pwmRampSpeed;

    /// Used to calculate the ramp dt.
    uint32_t prevTime = 0;

    /// The PWM pin that the servo is attached to.
    tap::gpio::Pwm::Pin servoPin;
};  // class Servo

}  // namespace motor

}  // namespace tap

#endif  // TAPROOT_SERVO_HPP_
