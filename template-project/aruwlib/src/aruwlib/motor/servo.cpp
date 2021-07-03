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

#include "servo.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/drivers.hpp"
#include "aruwlib/errors/create_errors.hpp"

namespace aruwlib
{
namespace motor
{
Servo::Servo(
    Drivers *drivers,
    aruwlib::gpio::Pwm::Pin pwmPin,
    float maximumPwm,
    float minimumPwm,
    float pwmRampSpeed)
    : drivers(drivers),
      pwmOutputRamp(0.0f),
      maxPwm(aruwlib::algorithms::limitVal<float>(maximumPwm, 0.0f, 1.0f)),
      minPwm(aruwlib::algorithms::limitVal<float>(minimumPwm, 0.0f, 1.0f)),
      pwmRampSpeed(pwmRampSpeed),
      prevTime(0),
      servoPin(pwmPin)
{
    if (maxPwm < minPwm)
    {
        minPwm = 0.0f;
        maxPwm = 1.0f;
        RAISE_ERROR(
            drivers,
            "min servo PWM > max servo PWM",
            errors::Location::SERVO,
            errors::ServoErrorType::INVALID_ADD);
    }
}

void Servo::setTargetPwm(float pwm)
{
    pwmOutputRamp.setTarget(aruwlib::algorithms::limitVal<float>(pwm, minPwm, maxPwm));
    prevTime = aruwlib::arch::clock::getTimeMilliseconds();
}

void Servo::updateSendPwmRamp()
{
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    pwmOutputRamp.update(pwmRampSpeed * (currTime - prevTime));
    prevTime = currTime;
    currentPwm = pwmOutputRamp.getValue();
    drivers->pwm.write(pwmOutputRamp.getValue(), servoPin);
}

float Servo::getPWM() const { return currentPwm; }

float Servo::getMinPWM() const { return minPwm; }

float Servo::getMaxPWM() const { return maxPwm; }

bool Servo::isRampTargetMet() const { return pwmOutputRamp.isTargetReached(); }

}  // namespace motor

}  // namespace aruwlib
