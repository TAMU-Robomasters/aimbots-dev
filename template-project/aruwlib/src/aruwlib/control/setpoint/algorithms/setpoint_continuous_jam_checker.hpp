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

#ifndef MOTOR_JAM_DETECTOR_
#define MOTOR_JAM_DETECTOR_

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/conditional_timer.hpp"
#include "aruwlib/control/setpoint/interfaces/setpoint_subsystem.hpp"

namespace aruwlib
{
namespace control
{
namespace setpoint
{
/**
 * A functor (function object) to be used for setpoint subsystem jam detection.
 *
 * Uses a conditional timeout to continuously check if the subsystem's current
 * position is within an acceptable range of the desired position. If it
 * is outside of the acceptable range for too long then the timeout times
 * out and the subsystem is considered jammed.
 */
class SetpointContinuousJamChecker
{
public:
    /**
     * @param[in] setpointSubsystem: the setpoint subsystem to do jam checking
     *      on.
     * @param[in] distanceTolerance: the acceptable distance between the setpoint and
     *      current position.
     * @param[in] temporalTolerance: the maximum amount of time the distance can be
     *      greater than the distance tolerance before the system is considered jammed.
     */
    SetpointContinuousJamChecker(
        SetpointSubsystem* setpointSubsystem,
        float distanceTolerance,
        uint32_t temporalTolerance)
        : setpointSubsystem(setpointSubsystem),
          jamTimeout(temporalTolerance),
          distanceTolerance(distanceTolerance)
    {
    }

    /**
     * Resets the jam timer
     */
    void restart() { jamTimeout.restart(); }

    /**
     * Update subsystem jam detection and check whether subsystem is jammed.
     * If subsystem is within distance tolerance of desired value then timer is
     * reset, even if at the time of call it would have expired, as being within
     * tolerance is checked first.
     *
     * @note Should be called once per subsystem refresh (it's like an execute)
     *
     * @return `true` if subsystem is jammed, `false` otherwise
     */
    inline bool check()
    {
        bool withinTolerance = aruwlib::algorithms::compareFloatClose(
            setpointSubsystem->getCurrentValue(),
            setpointSubsystem->getSetpoint(),
            distanceTolerance);
        return jamTimeout.execute(!withinTolerance);
    }

private:
    SetpointSubsystem* setpointSubsystem;
    aruwlib::arch::ConditionalMilliTimer jamTimeout;
    float distanceTolerance;
};  // SetpointContinuousJamChecker

}  // namespace setpoint

}  // namespace control

}  // namespace aruwlib

#endif  // MOTOR_JAM_DETECTOR_
