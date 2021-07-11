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

#include "unjam_command.hpp"

#include "aruwlib/control/setpoint/interfaces/setpoint_subsystem.hpp"

namespace aruwlib
{
namespace control
{
namespace setpoint
{
class SetpointSubsystem;  // forward declaration

UnjamCommand::UnjamCommand(
    SetpointSubsystem* setpointSubsystem,
    float agitatorMaxUnjamAngle,
    uint32_t agitatorMaxWaitTime)
    : currUnjamstate(AGITATOR_UNJAM_BACK),
      agitatorUnjamRotateTimeout(0),
      salvationTimeout(0),
      agitatorMaxWaitTime(agitatorMaxWaitTime),
      setpointSubsystem(setpointSubsystem),
      agitatorUnjamAngleMax(agitatorMaxUnjamAngle),
      currAgitatorUnjamAngle(0.0f),
      agitatorSetpointBeforeUnjam(0.0f)
{
    if (agitatorMaxUnjamAngle < MIN_AGITATOR_UNJAM_ANGLE)
    {
        agitatorUnjamAngleMax = MIN_AGITATOR_UNJAM_ANGLE;
    }
    this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(setpointSubsystem));
    salvationTimeout.stop();
    agitatorUnjamRotateTimeout.stop();
}

void UnjamCommand::initialize()
{
    agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

    // define a random unjam angle between [MIN_AGITATOR_UNJAM_ANGLE, agitatorUnjamAngleMax]
    const float minUnjamAngle =
        agitatorUnjamAngleMax <= MIN_AGITATOR_UNJAM_ANGLE ? 0 : MIN_AGITATOR_UNJAM_ANGLE;
    float randomUnjamAngle = fmodf(rand(), agitatorUnjamAngleMax - minUnjamAngle) + minUnjamAngle;

    // subtract this angle from the current angle to rotate agitator backwards
    currAgitatorUnjamAngle = setpointSubsystem->getCurrentValue() - randomUnjamAngle;

    // store the current setpoint angle to be referenced later
    agitatorSetpointBeforeUnjam = setpointSubsystem->getSetpoint();
    currUnjamstate = AGITATOR_UNJAM_BACK;

    salvationTimeout.restart(SALVATION_TIMEOUT_MS);
}

void UnjamCommand::execute()
{
    if (salvationTimeout.execute())
    {
        currAgitatorUnjamAngle = agitatorSetpointBeforeUnjam - 2 * aruwlib::algorithms::PI;
        salvationTimeout.stop();
        agitatorUnjamRotateTimeout.restart(SALVATION_UNJAM_BACK_WAIT_TIME);
        currUnjamstate = AGITATOR_SALVATION_UNJAM_BACK;
    }

    switch (currUnjamstate)
    {
        case AGITATOR_SALVATION_UNJAM_BACK:
        {
            setpointSubsystem->setSetpoint(currAgitatorUnjamAngle);
            if (agitatorUnjamRotateTimeout.isExpired() ||
                fabsf(setpointSubsystem->getCurrentValue() - setpointSubsystem->getSetpoint()) <
                    SETPOINT_TOLERANCE)
            {
                currUnjamstate = FINISHED;
            }
            break;
        }
        case AGITATOR_UNJAM_BACK:
        {
            setpointSubsystem->setSetpoint(currAgitatorUnjamAngle);
            if (agitatorUnjamRotateTimeout.isExpired() ||
                fabsf(setpointSubsystem->getCurrentValue() - setpointSubsystem->getSetpoint()) <
                    SETPOINT_TOLERANCE)
            {  // either the timeout has been triggered or the agitator has reached the setpoint
                // define a random time that the agitator will take to rotate forwards.
                agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

                // reset the agitator
                currUnjamstate = AGITATOR_UNJAM_RESET;
            }
            break;
        }
        case AGITATOR_UNJAM_RESET:  // this is different than just agitator_rotate_command
        {
            // reset the angle to what it was before unjamming
            setpointSubsystem->setSetpoint(agitatorSetpointBeforeUnjam);
            // the agitator is still jammed
            if (agitatorUnjamRotateTimeout.isExpired())
            {
                // restart the timeout
                agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

                // define a new random angle, which will be used in the unjam back state
                const float minUnjamAngle = agitatorUnjamAngleMax <= MIN_AGITATOR_UNJAM_ANGLE
                                                ? 0
                                                : MIN_AGITATOR_UNJAM_ANGLE;
                float randomUnjamAngle =
                    fmodf(rand(), agitatorUnjamAngleMax - minUnjamAngle) + minUnjamAngle;

                currAgitatorUnjamAngle = agitatorSetpointBeforeUnjam - randomUnjamAngle;

                currUnjamstate = AGITATOR_UNJAM_BACK;
            }
            else if (
                fabsf(setpointSubsystem->getCurrentValue() - setpointSubsystem->getSetpoint()) <
                SETPOINT_TOLERANCE)
            {
                currUnjamstate = FINISHED;
            }
            break;
        }
        case FINISHED:  // this could be only two states, but its simpler to debug with three
        {
            break;
        }
    }
}

void UnjamCommand::end(bool) { setpointSubsystem->clearJam(); }

bool UnjamCommand::isFinished(void) const { return currUnjamstate == FINISHED; }

}  // namespace setpoint

}  // namespace control

}  // namespace aruwlib
