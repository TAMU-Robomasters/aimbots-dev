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

#ifndef AGITATOR_ABSOLUTE_ROTATE_COMMAND_HPP_
#define AGITATOR_ABSOLUTE_ROTATE_COMMAND_HPP_

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
 * A command that uses an `SetpointSubsystem` to rotate to the same
 * angle everytime, attemping to rotate at the given angular velocity.
 * (Consistency doesn't work across motor disconnects). This command
 * ends immediately if the agitator is jammed, and upon ending will
 * stop the connected agitator by setting its target position to its
 * current position.
 *
 * Agitator angles are relative, and the "0"-angle is changed when
 * the agitator is calibrated.
 */
class MoveAbsoluteCommand : public tap::control::Command
{
public:
    /**
     * @param[in] setpointSubsystem the subsystem this command manipulates.
     * @param[in] setpoint the target value the controlled variable
     *  should attempt to reach
     * @param[in] speed The angular speed the agitator should
     *  attempt to move at in milliradians/second
     * @param[in] setpointTolerance the command will consider the target angle
     *  as reached when it's distance to the target is within this value
     * @param[in] shouldAutomaticallyClearJam the command will clear the subsystem's
     *  jam state without any unjamming performed
     */
    explicit MoveAbsoluteCommand(
        tap::control::setpoint::SetpointSubsystem* setpointSubsystem,
        float targetAngle,
        uint32_t agitatorRotateSpeed,
        float setpointTolerance,
        bool shouldAutomaticallyClearJam);

    const char* getName() const override { return "open hopper lid"; }

    bool isReady() override { return !setpointSubsystem->isJammed(); }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

protected:
    tap::control::setpoint::SetpointSubsystem* setpointSubsystem;

private:
    /* target angle for the agitator to reach when command is called.*/
    float targetAngle;

    tap::algorithms::Ramp rampToTargetAngle;

    /**
     * The angular speed the agitator should attempt to move at in
     * milliradians/second.
     */
    uint32_t agitatorRotateSpeed;

    float agitatorSetpointTolerance;

    uint32_t agitatorPrevRotateTime;

    bool automaticallyClearJam;
};  // class MoveAbsoluteCommand

}  // namespace setpoint

}  // namespace control

}  // namespace tap

#endif  // AGITATOR_ABSOLUTE_ROTATE_COMMAND_HPP_
