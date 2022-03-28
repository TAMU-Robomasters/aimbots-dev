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

#ifndef TAPROOT_CHASSIS_SUBSYSTEM_INTERFACE_HPP_
#define TAPROOT_CHASSIS_SUBSYSTEM_INTERFACE_HPP_

#include "tap/motor/dji_motor.hpp"

#include "../subsystem.hpp"

namespace tap::control::chassis
{
/**
 * @brief Abstract interface for a robot chassis.
 */
class ChassisSubsystemInterface : public Subsystem
{
public:
    ChassisSubsystemInterface(Drivers *drivers) : Subsystem(drivers) {}

    /**
     * @return the number of chassis motors
     */
    virtual inline int getNumChassisMotors() const = 0;

    virtual inline int16_t getLeftFrontRpmActual() const = 0;
    virtual inline int16_t getLeftBackRpmActual() const = 0;
    virtual inline int16_t getRightFrontRpmActual() const = 0;
    virtual inline int16_t getRightBackRpmActual() const = 0;

    /**
     * @return `true` iff all motors are online
     */
    virtual inline bool allMotorsOnline() const = 0;
};
}  // namespace tap::control::chassis

#endif  // TAPROOT_CHASSIS_SUBSYSTEM_INTERFACE_HPP_
