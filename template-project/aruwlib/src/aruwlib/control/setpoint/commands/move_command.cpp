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

#include "move_command.hpp"

#include "aruwlib/architecture/clock.hpp"

namespace aruwlib
{
namespace control
{
namespace setpoint
{
MoveCommand::MoveCommand(
    SetpointSubsystem* agitator,
    float agitatorAngleChange,
    uint32_t agitatorRotateTime,
    uint32_t agitatorPauseAfterRotateTime,
    bool agitatorSetToFinalAngle,
    float setpointTolerance)
    : setpointSubsystem(agitator),
      agitatorTargetAngleChange(agitatorAngleChange),
      rampToTargetAngle(0.0f),
      agitatorDesiredRotateTime(agitatorRotateTime),
      agitatorMinRotatePeriod(agitatorRotateTime + agitatorPauseAfterRotateTime),
      agitatorMinRotateTimeout(agitatorRotateTime + agitatorPauseAfterRotateTime),
      agitatorSetpointTolerance(setpointTolerance),
      agitatorPrevRotateTime(0),
      agitatorSetToFinalAngle(agitatorSetToFinalAngle)
{
    this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(agitator));
}

void MoveCommand::initialize()
{
    // set the ramp start and target angles
    rampToTargetAngle.setTarget(setpointSubsystem->getSetpoint() + agitatorTargetAngleChange);

    rampToTargetAngle.setValue(setpointSubsystem->getCurrentValue());

    agitatorPrevRotateTime = aruwlib::arch::clock::getTimeMilliseconds();
}

void MoveCommand::execute()
{
    // update the agitator setpoint ramp
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    rampToTargetAngle.update(
        (currTime - agitatorPrevRotateTime) * agitatorTargetAngleChange /
        static_cast<float>(agitatorDesiredRotateTime));
    agitatorPrevRotateTime = currTime;
    setpointSubsystem->setSetpoint(rampToTargetAngle.getValue());
}

void MoveCommand::end(bool)
{
    // if the agitator is not interrupted, then it exited normally
    // (i.e. reached the desired angle) and is not jammed. If it is
    // jammed we thus want to set the agitator angle to the current angle,
    // so the motor does not attempt to keep rotating forward (and possible stalling)
    if (setpointSubsystem->isJammed() || !agitatorSetToFinalAngle)
    {
        setpointSubsystem->setSetpoint(setpointSubsystem->getCurrentValue());
    }
    else
    {
        setpointSubsystem->setSetpoint(rampToTargetAngle.getTarget());
    }
}

bool MoveCommand::isFinished() const
{
    // The subsystem is jammed, or it is within the setpoint tolerance, the ramp is
    // finished, and the minimum rotate time is expired.
    return setpointSubsystem->isJammed() ||
           (fabsf(setpointSubsystem->getCurrentValue() - setpointSubsystem->getSetpoint()) <
                agitatorSetpointTolerance &&
            rampToTargetAngle.isTargetReached() && agitatorMinRotateTimeout.isExpired());
}

}  // namespace setpoint

}  // namespace control

}  // namespace aruwlib
