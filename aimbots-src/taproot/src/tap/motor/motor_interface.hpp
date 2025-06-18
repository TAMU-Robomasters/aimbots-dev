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

#ifndef TAPROOT_MOTOR_INTERFACE_HPP_
#define TAPROOT_MOTOR_INTERFACE_HPP_

#include <cstdint>

namespace tap::motor
{
class MotorInterface
{
public:
    virtual void initialize() = 0;
    virtual int64_t getEncoderUnwrapped() const = 0;
    virtual uint16_t getEncoderWrapped() const = 0;
    virtual void setDesiredOutput(int32_t desiredOutput) = 0;
    virtual bool isMotorOnline() const = 0;
    virtual int16_t getOutputDesired() const = 0;
    virtual int8_t getTemperature() const = 0;
    virtual int16_t getTorque() const = 0;
    virtual int16_t getShaftRPM() const = 0;
};

}  // namespace tap::motor

#endif  // TAPROOT_MOTOR_INTERFACE_HPP_
