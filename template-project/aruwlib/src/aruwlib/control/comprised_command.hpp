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

#ifndef __COMPRISED_COMMAND_HPP__
#define __COMPRISED_COMMAND_HPP__

#include "command.hpp"
#include "command_scheduler.hpp"

namespace aruwlib
{
class Drivers;
namespace control
{
/**
 * A class with all the features of a Command but with the addition of
 * a CommandScheduler that can be used to schedule multiple
 * Commands inside a single Command. If you are making a comprised command,
 * operations in this Command should operate at a high level. In essence,
 * a comprised acts as a vessel for a state machine that when it wants
 * to change the state of the robot, it adds/removes commands to its
 * command scheduler instead of directly interacting with a subsystem.
 *
 * For example, consider this use case: You have a Command that actuates
 * a piston to grab something and another Command that flips a wrist that
 * has the piston out. It would be nice to reuse these Commands and make
 * a Command that flips the wrist out and then actuates the piston in quick
 * succession. To do so, you can create a ComprisedCommand that consists of
 * the two Commands described above. In this ComprisedCommand, first
 * schedule the Command that flips the wrist out, then when that Command
 * is done, schedule the Command that actuates the piston.
 *
 * When you are using this the `comprisedCommandScheduler`, be sure to
 * register Subsystems and add Subsystem dependencies for the Commands that
 * will be added to the scheduler.
 */
class ComprisedCommand : public Command
{
public:
    ComprisedCommand(Drivers *drivers) : Command(), comprisedCommandScheduler(drivers) {}

protected:
    CommandScheduler comprisedCommandScheduler;
};

}  // namespace control

}  // namespace aruwlib

#endif
