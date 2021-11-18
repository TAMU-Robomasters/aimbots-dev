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

#include "move_absolute_command.hpp"

#include "tap/architecture/clock.hpp"

namespace tap
{
namespace control
{
namespace setpoint
{
MoveAbsoluteCommand::MoveAbsoluteCommand(
    SetpointSubsystem* setpointSubsystem,
    float targetAngle,
    uint32_t agitatorRotateSpeed,
    float setpointTolerance,
    bool automaticallyClearJam)
    : setpointSubsystem(setpointSubsystem),
      targetAngle(targetAngle),
      agitatorRotateSpeed(agitatorRotateSpeed),
      agitatorSetpointTolerance(setpointTolerance),
      automaticallyClearJam(automaticallyClearJam)
{
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(setpointSubsystem));
}

void MoveAbsoluteCommand::initialize()
{
    rampToTargetAngle.setTarget(targetAngle);
    rampToTargetAngle.setValue(setpointSubsystem->getCurrentValue());
    agitatorPrevRotateTime = tap::arch::clock::getTimeMilliseconds();
}

void MoveAbsoluteCommand::execute()
{
    // If the agitator is jammed, set the setpoint to the current angle. Necessary since
    // derived classes may choose to overwrite the `isFinished` function and so for motor safety
    // we do this.
    if (setpointSubsystem->isJammed())
    {
        setpointSubsystem->setSetpoint(setpointSubsystem->getCurrentValue());
        return;
    }

    // We can assume that agitator is connected, otherwise end will be called.
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    // Divide by 1'000'000 to get radians because agitatorRotateSpeed is in milliRadians/second
    // and time interval is in milliseconds. (milliseconds * 1/1000 (second/millisecond) *
    // (milliradians/second) * 1/1000 (radians/milliradian) = 1/1'000'000 as our conversion
    rampToTargetAngle.update(
        (static_cast<float>(currTime - agitatorPrevRotateTime) * agitatorRotateSpeed) /
        1'000'000.0f);
    agitatorPrevRotateTime = currTime;

    setpointSubsystem->setSetpoint(rampToTargetAngle.getValue());
}

void MoveAbsoluteCommand::end(bool)
{
    // When this command ends we want to make sure to set the agitator's target angle
    // to it's current angle so it doesn't keep trying to move, especially if it's jammed
    // or reached the end of its range of motion.
    setpointSubsystem->setSetpoint(setpointSubsystem->getCurrentValue());
    if (automaticallyClearJam)
    {
        setpointSubsystem->clearJam();
    }
}

bool MoveAbsoluteCommand::isFinished() const
{
    // Command is finished if we've reached target, lost connection to agitator, or
    // if our agitator is jammed.
    return (fabsf(setpointSubsystem->getCurrentValue() - rampToTargetAngle.getTarget()) <
            agitatorSetpointTolerance) ||
           !setpointSubsystem->isOnline() || setpointSubsystem->isJammed();
}

}  // namespace setpoint

}  // namespace control

}  // namespace tap
