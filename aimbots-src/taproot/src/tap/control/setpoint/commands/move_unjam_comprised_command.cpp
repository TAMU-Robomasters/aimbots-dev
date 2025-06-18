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

#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/setpoint/interfaces/setpoint_subsystem.hpp"

#include "move_command.hpp"
#include "unjam_command.hpp"

using namespace tap::control;

namespace tap
{
namespace control
{
namespace setpoint
{
MoveUnjamComprisedCommand::MoveUnjamComprisedCommand(
    tap::Drivers* drivers,
    SetpointSubsystem* setpointSubsystem,
    float moveDisplacement,
    uint32_t moveTime,
    uint32_t pauseAfterMoveTime,
    bool setToTargetOnEnd,
    float setpointTolerance,
    float unjamDisplacement,
    float unjamThreshold,
    uint32_t maxUnjamWaitTime,
    uint_fast16_t unjamCycleCount)
    : tap::control::ComprisedCommand(drivers),
      setpointSubsystem(setpointSubsystem),
      agitatorRotateCommand(
          setpointSubsystem,
          moveDisplacement,
          moveTime,
          pauseAfterMoveTime,
          setToTargetOnEnd,
          setpointTolerance),
      agitatorUnjamCommand(
          setpointSubsystem,
          unjamDisplacement,
          unjamThreshold,
          maxUnjamWaitTime,
          unjamCycleCount),
      unjamSequenceCommencing(false),
      agitatorDisconnectFault(false)
{
    this->comprisedCommandScheduler.registerSubsystem(setpointSubsystem);
    this->addSubsystemRequirement(setpointSubsystem);
}

void MoveUnjamComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(&agitatorRotateCommand);
    unjamSequenceCommencing = false;
}

void MoveUnjamComprisedCommand::execute()
{
    // If setpointSubsystem has disconnected, set flag to remember this when isFinished() and end()
    // are called. (This check can't be done in isFinished() because it's const function)
    if (!setpointSubsystem->isOnline())
    {
        agitatorDisconnectFault = true;
    }
    else
    {
        // If setpointSubsystem isn't disconnected run our normal logic
        if (setpointSubsystem->isJammed() && !unjamSequenceCommencing)
        {
            // when the setpointSubsystem is jammed, add the agitatorUnjamCommand
            // the to scheduler. The rotate forward command will be automatically
            // unscheduled.
            unjamSequenceCommencing = true;
            this->comprisedCommandScheduler.addCommand(&agitatorUnjamCommand);
        }
        this->comprisedCommandScheduler.run();
    }
}

void MoveUnjamComprisedCommand::end(bool interrupted)
{
    // The command could have also been interrupted by loss of setpointSubsystem
    // connection. Account for that by OR'ing them.
    interrupted |= agitatorDisconnectFault;
    // agitatorDisconnect has been acknowledged, regardless of previous state
    // set it back to false
    agitatorDisconnectFault = false;

    this->comprisedCommandScheduler.removeCommand(&agitatorUnjamCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&agitatorRotateCommand, interrupted);
}

bool MoveUnjamComprisedCommand::isFinished() const
{
    return (!unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&agitatorRotateCommand)) ||
           (unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&agitatorUnjamCommand)) ||
           agitatorDisconnectFault;
}

}  // namespace setpoint

}  // namespace control

}  // namespace tap
