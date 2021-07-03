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

#include "aruwlib/control/setpoint/commands/move_unjam_comprised_command.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/control/command_scheduler.hpp"
#include "aruwlib/control/setpoint/interfaces/setpoint_subsystem.hpp"

#include "move_command.hpp"
#include "unjam_command.hpp"

using namespace aruwlib::control;

namespace aruwlib
{
namespace control
{
namespace setpoint
{
MoveUnjamComprisedCommand::MoveUnjamComprisedCommand(
    aruwlib::Drivers* drivers,
    SetpointSubsystem* setpointSubsystem,
    float agitatorChangeAngle,
    float maxUnjamAngle,
    uint32_t agitatorRotateTime,
    uint32_t agitatorPauseAfterRotateTime)
    : aruwlib::control::ComprisedCommand(drivers),
      setpointSubsystem(setpointSubsystem),
      agitatorRotateCommand(
          setpointSubsystem,
          agitatorChangeAngle,
          agitatorRotateTime,
          agitatorPauseAfterRotateTime,
          false),
      agitatorUnjamCommand(setpointSubsystem, maxUnjamAngle),
      unjamSequenceCommencing(false),
      agitatorDisconnectFault(false)
{
    this->comprisedCommandScheduler.registerSubsystem(setpointSubsystem);
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(setpointSubsystem));
}

void MoveUnjamComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&agitatorRotateCommand));
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
            this->comprisedCommandScheduler.addCommand(
                dynamic_cast<Command*>(&agitatorUnjamCommand));
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

    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&agitatorUnjamCommand),
        interrupted);
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&agitatorRotateCommand),
        interrupted);
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

}  // namespace aruwlib
