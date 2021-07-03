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

#include "subsystem.hpp"

#include "command_scheduler.hpp"

namespace aruwlib
{
namespace control
{
Subsystem::Subsystem(Drivers* drivers)
    : drivers(drivers),
      defaultCommand(nullptr),
      globalIdentifier(CommandScheduler::constructSubsystem(this))
{
}

Subsystem::~Subsystem() { CommandScheduler::destructSubsystem(this); }

void Subsystem::setDefaultCommand(Command* command)
{
    if (command != nullptr)
    {
        defaultCommand = command;
    }
}

const char* Subsystem::getName() { return "Subsystem"; }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
Subsystem::Subsystem()
    : drivers(nullptr),
      defaultCommand(nullptr),
      globalIdentifier(CommandScheduler::constructSubsystem(this))
{
}
#endif
}  // namespace control

}  // namespace aruwlib
