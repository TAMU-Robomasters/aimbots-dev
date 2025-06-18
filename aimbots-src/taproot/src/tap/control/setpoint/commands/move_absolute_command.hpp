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

#ifndef TAPROOT_MOVE_ABSOLUTE_COMMAND_HPP_
#define TAPROOT_MOVE_ABSOLUTE_COMMAND_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"
#include "tap/control/setpoint/interfaces/setpoint_subsystem.hpp"

namespace tap
{
namespace control
{
namespace setpoint
{
/**
 * A command that uses an `SetpointSubsystem` to move to the same
 * position/value everytime, attemping to rotate at the given velocity.
 * (Coordinates may shift if motor disconnects). This command
 * ends immediately if the agitator is jammed, and upon ending will
 * stop the connected agitator by setting its target position to its
 * current position.
 *
 * Subsystem values are relative, and the "0"-value is changed when
 * the agitator is calibrated.
 */
class MoveAbsoluteCommand : public tap::control::Command
{
public:
    /**
     * @param[in] setpointSubsystem the subsystem this command manipulates.
     * @param[in] setpoint the target value the controlled variable
     *      should reach
     * @param[in] speed The speed the subsystem should attempt to move its value
     *      at in  units/second (where "units" are the same as those the setpoint uses)
     * @param[in] setpointTolerance the command will consider the target value
     *      as reached when it's distance to the target is within this value
     * @param[in] shouldAutomaticallyClearJam the command will clear the subsystem's
     *      jam state without any unjamming performed
     * @param[in] setSetpointToTargetOnEnd the command will set the subsystem's setpoint
     *      to the target value when ending if true, otherwise it will set the setpoint to
     *      the subsystem's current value.
     */
    explicit MoveAbsoluteCommand(
        tap::control::setpoint::SetpointSubsystem* setpointSubsystem,
        float setpoint,
        float speed,
        float setpointTolerance,
        bool shouldAutomaticallyClearJam,
        bool setSetpointToTargetOnEnd);

    const char* getName() const override { return "move absolute"; }

    bool isReady() override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

protected:
    tap::control::setpoint::SetpointSubsystem* setpointSubsystem;

private:
    /**
     * target value for the subsystem to reach when command is called.
     */
    float setpoint;

    tap::algorithms::Ramp rampToSetpoint;

    /**
     * The speed the subsystem should attempt to move at in
     * setpoint-units / second.
     */
    float speed;

    float setpointTolerance;

    uint32_t prevMoveTime;

    bool automaticallyClearJam;

    bool setSetpointToTargetOnEnd;
};  // class MoveAbsoluteCommand

}  // namespace setpoint

}  // namespace control

}  // namespace tap

#endif  // TAPROOT_MOVE_ABSOLUTE_COMMAND_HPP_
