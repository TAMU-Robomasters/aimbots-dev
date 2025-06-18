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

#ifndef TAPROOT_SETPOINT_SUBSYSTEM_HPP_
#define TAPROOT_SETPOINT_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

namespace tap
{
// Forward declaration
class Drivers;

namespace control
{
namespace setpoint
{
/**
 * An abstract class (usable as an interface) describing the functionalities
 * of a subsystem which uses a positional controller to rotate something.
 *
 * Any mention of `position` in the documentation for this class does not
 * necessarily refer to a physical location in space, rather it refers to
 * a quantity describing whatever is being positionally controlled by the
 * subsystem.
 */
class SetpointSubsystem : public virtual tap::control::Subsystem
{
public:
    /**
     * @return the subsystem's setpoint: the desired value of whatever is
     * being controlled.
     */
    virtual inline float getSetpoint() const = 0;

    /**
     * Sets the desired position of the subsystem, relative to where
     * it was calibrated.
     *
     * @param[in] newValue: the new desired value the subsystem will try to
     *  reach.
     */
    virtual inline void setSetpoint(float newAngle) = 0;

    /**
     * @return The current value of the controlled variable.
     */
    virtual float getCurrentValue() const = 0;

    /**
     * @return the jamming tolerance. This is the maximum distance between
     *      the ideal setpoint and current value of the controlled variable
     *      at which the subsystem will never consider itself jammed.
     */
    virtual float getJamSetpointTolerance() const = 0;

    /**
     * Attempts to calibrate the subsystem at the current position, such that
     * `getCurrentValue` will return 0 units at this position.
     *
     * @return `true` if the subsystem has been successfully calibrated, `false`
     *  otherwise.
     */
    virtual bool calibrateHere() = 0;

    /**
     * @return `true` if the subsystem unjam timer has expired, signaling that the subsystem
     *  has jammed, `false` otherwise.
     */
    virtual bool isJammed() = 0;

    /**
     * Call to clear the jam flag of the subsystem, indicating that the jam has been solved.
     * @todo At some point we should move the unjam command logic into the subsystems
     */
    virtual void clearJam() = 0;

    /**
     * @return `true` if the subsystem has been calibrated
     */
    virtual inline bool isCalibrated() = 0;

    /**
     * @return `true` if the subsystem is online (i.e.: is connected)
     */
    virtual inline bool isOnline() = 0;

    /**
     * @return the velocity of the subsystem (i.e.: the rate of change of the
     *      controlled variable's value).
     */
    virtual inline float getVelocity() = 0;

};  // class SetpointSubsystem

}  // namespace setpoint

}  // namespace control

}  // namespace tap

#endif  // TAPROOT_SETPOINT_SUBSYSTEM_HPP_
