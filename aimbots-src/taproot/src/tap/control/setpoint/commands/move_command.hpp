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

#ifndef TAPROOT_MOVE_COMMAND_HPP_
#define TAPROOT_MOVE_COMMAND_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"
#include "tap/control/setpoint/interfaces/setpoint_subsystem.hpp"
#include "tap/drivers.hpp"

#include "modm/math/filter/pid.hpp"

namespace tap
{
namespace control
{
namespace setpoint
{
/**
 * Displaces the connected subsystem some value in some desired time. Currently
 * pass in a displacement and time to move and it uses `tap::arch::getTimeMilliseconds()`
 * to determine the speed to move at.
 *
 * Ends if subsystem is offline.
 */
class MoveCommand : public tap::control::Command
{
public:
    /**
     * @param[in] setpointSubsystem The subsystem associated with the rotate command.
     * @param[in] targetDisplacement The desired change in subsystem value in subsystem units.
     * @param[in] moveTime The time it takes to move the subsystem to the desired
     *      value in milliseconds.
     * @param[in] pauseAfterMoveTime Time in milliseconds that the command will wait after
     *      reaching the target displacement before the command is considered complete.
     * @param[in] setToTargetOnEnd if `true` the command will set the subsystem setpoint
     *      to the ideal target value on an uninterrupted `end()` if subsystem is online and
     *      unjammed, otherwise the subsystem will always set the setpoint to the its current value
     *      on `end()`.
     * @param[in] setpointTolerance The difference between current and desired value when the
     *      command will be considered to be completed (used in the `isFinished` function). Uses
     *      the same units as the subsystem's setpoint.
     * @attention the ramp value is calculated by finding the rotation speed
     *      (\f$targetDisplacement / moveTime\f$), and then multiplying this by
     *      the period (how often the ramp is called)
     */
    MoveCommand(
        SetpointSubsystem* setpointSubsystem,
        float targetDisplacement,
        uint32_t moveTime,
        uint32_t pauseAfterMoveTime,
        bool setToTargetOnEnd,
        float setpointTolerance);

    const char* getName() const override { return "move command"; }

    bool isReady() override
    {
        return !setpointSubsystem->isJammed() && setpointSubsystem->isOnline();
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    SetpointSubsystem* setpointSubsystem;

    /**
     * The target setpoint displacement from the current setpoint everytime this command
     * is run.
     */
    float targetDisplacement;

    tap::algorithms::Ramp rampToTargetValue;

    /**
     * The time you want the subsystem to take to rotate to the desired value, in milliseconds.
     */
    uint32_t moveTime;

    tap::arch::MilliTimeout minMoveTimeout;

    uint32_t pauseAfterMoveTime;

    float setpointTolerance;

    uint32_t previousMoveTime;

    bool setToTargetOnEnd;
};  // class MoveCommand

}  // namespace setpoint

}  // namespace control

}  // namespace tap

#endif  // TAPROOT_MOVE_COMMAND_HPP_
