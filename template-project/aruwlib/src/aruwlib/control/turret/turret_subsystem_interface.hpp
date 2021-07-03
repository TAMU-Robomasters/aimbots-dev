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

#ifndef TURRET_SUBSYSTEM_INTERFACE_
#define TURRET_SUBSYSTEM_INTERFACE_

#include "aruwlib/algorithms/contiguous_float.hpp"

#include "../subsystem.hpp"

namespace aruwlib::control::turret
{
/**
 * Interface for a generic turret motor with a pitch and yaw motor.
 */
class TurretSubsystemInterface : public Subsystem
{
public:
    explicit TurretSubsystemInterface(Drivers *drivers) : Subsystem(drivers) {}

    /**
     * @return the desired yaw value value of whatever is being controlled.
     */
    virtual inline float getYawSetpoint() const = 0;
    /**
     * @see getYawSetpoint
     */
    virtual inline float getPitchSetpoint() const = 0;

    /**
     * Sets the desired yaw position of the turret.
     *
     * @param[in] newValue: the new desired value the turret will try and reach
     */
    virtual void setYawSetpoint(float newAngle) = 0;
    /**
     * @see setPitchSetpoint
     */
    virtual void setPitchSetpoint(float newAngle) = 0;

    /**
     * @return The current value of the turret's physical yaw.
     */
    virtual const aruwlib::algorithms::ContiguousFloat &getCurrentYawValue() const = 0;
    /**
     * @see getCurrentYawValue
     */
    virtual const aruwlib::algorithms::ContiguousFloat &getCurrentPitchValue() const = 0;

    /**
     * @return `true` if the turret is online (i.e.: is connected)
     */
    virtual inline bool isOnline() const = 0;

    /**
     * @return the velocity of the turret's physical yaw motor(s)
     */
    virtual inline float getYawVelocity() const = 0;
    /**
     * @see getYawVelocity
     */
    virtual inline float getPitchVelocity() const = 0;

    /**
     * @see getPitchAngleFromCenter
     */
    virtual float getYawAngleFromCenter() const = 0;

    /**
     * @return An angle between [-180, 180] that is the angle difference of the pitch's current
     *      value and the center value as defined by a subclass.
     */
    virtual float getPitchAngleFromCenter() const = 0;
};
}  // namespace aruwlib::control::turret

#endif  // TURRET_SUBSYSTEM_INTERFACE_
