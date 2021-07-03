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

#ifndef AGITATOR_SHOOT_COMPRISED_COMMAND_HPP_
#define AGITATOR_SHOOT_COMPRISED_COMMAND_HPP_

#include "aruwlib/control/comprised_command.hpp"

#include "move_command.hpp"
#include "unjam_command.hpp"

namespace aruwlib
{
namespace control
{
namespace setpoint
{
// Forward declarations
class SetpointSubsystem;

/**
 * A comprised command that combines the agitator unjam and rotate commands and provides
 * unjam monitoring to perform a single agitator rotation with unjamming if necessary.
 */
class MoveUnjamComprisedCommand : public aruwlib::control::ComprisedCommand
{
public:
    /**
     * @param[in] agitator The agitator to interact with.
     * @param[in] agitatorChangeAngle The angle in radians that the agitator should rotate.
     * @param[in] maxUnjamAngle See `AgitatorUnJamCommand`'s constructor for more details,
     *      passed on directly to this command's constructor.
     * @param[in] agitatorRotateTime The time it takes to rotate the agitator to the desired angle
     *      in milliseconds.
     * @param[in] agitatorPauseAfterRotateTime The time that the command will wait after rotating to
     *      the desired angle before the command is considered complete.
     */
    MoveUnjamComprisedCommand(
        aruwlib::Drivers* drivers,
        SetpointSubsystem* setpointSubsystem,
        float agitatorChangeAngle,
        float maxUnjamAngle,
        uint32_t agitatorRotateTime,
        uint32_t agitatorPauseAfterRotateTime);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "agitator shoot"; }

protected:
    SetpointSubsystem* setpointSubsystem;

    MoveCommand agitatorRotateCommand;

    UnjamCommand agitatorUnjamCommand;

    bool unjamSequenceCommencing;

    bool agitatorDisconnectFault;
};  // class MoveUnjamComprisedCommand

}  // namespace setpoint

}  // namespace control

}  // namespace aruwlib

#endif  // AGITATOR_SHOOT_COMPRISED_COMMAND_HPP_
