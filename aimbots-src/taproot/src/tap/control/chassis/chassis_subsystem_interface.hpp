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

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/motor/dji_motor.hpp"

#include "../subsystem.hpp"
#include "modm/math/matrix.hpp"

namespace tap::control::chassis
{
/**
 * @brief Abstract interface for a robot chassis.
 */
class ChassisSubsystemInterface : public Subsystem
{
public:
    ChassisSubsystemInterface(Drivers* drivers) : Subsystem(drivers) {}

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

    /**
     * @return The actual chassis velocity in chassis relative frame, as a vector <vx, vy, vz>,
     *      where vz is rotational velocity. This is the velocity calculated from the chassis's
     *      encoders. Units: m/s
     */
    virtual modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const = 0;

    /**
     * Transforms the chassis relative velocity of the form <vx, vy, vz> (where z is an
     * orientation) into world relative frame, given some particular chassis heading (z direction,
     * assumed to be in radians). Transforms the input matrix chassisRelativeVelocity. Units: m/s
     */
    static void getVelocityWorldRelative(
        modm::Matrix<float, 3, 1>& chassisRelativeVelocity,
        float chassisHeading)
    {
        modm::Matrix<float, 3, 3> transform;
        float headingCos = cosf(chassisHeading);
        float headingSin = sinf(chassisHeading);
        headingCos = tap::algorithms::compareFloatClose(headingCos, 0.0f, 1e-6) ? 0.0f : headingCos;
        headingSin = tap::algorithms::compareFloatClose(headingSin, 0.0f, 1e-6) ? 0.0f : headingSin;

        transform[0][0] = headingCos;
        transform[1][0] = headingSin;
        transform[2][0] = 0;
        transform[0][1] = -headingSin;
        transform[1][1] = headingCos;
        transform[2][1] = 0;
        transform[0][2] = 0;
        transform[1][2] = 0;
        transform[2][2] = 1;
        chassisRelativeVelocity = transform * chassisRelativeVelocity;
    }
};
}  // namespace tap::control::chassis

#endif  // TAPROOT_CHASSIS_SUBSYSTEM_INTERFACE_HPP_
