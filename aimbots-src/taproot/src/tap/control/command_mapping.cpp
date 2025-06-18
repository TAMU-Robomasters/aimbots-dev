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

#include "command_mapping.hpp"

#include <algorithm>

#include "tap/drivers.hpp"

namespace tap
{
namespace control
{
CommandMapping::CommandMapping(
    Drivers *drivers,
    const std::vector<Command *> cmds,
    const RemoteMapState &rms)
    : mapState(rms),
      mappedCommands(cmds),
      drivers(drivers)
{
    std::remove_if(mappedCommands.begin(), mappedCommands.end(), [](Command *c) {
        return c == nullptr;
    });
}

bool operator==(const CommandMapping &cm1, const CommandMapping &cm2)
{
    return (cm1.mapState == cm2.mapState) && (cm1.mappedCommands == cm2.mappedCommands);
}

bool mapStateEqual(const CommandMapping &cm1, const CommandMapping &cm2)
{
    // When inserting mappings into the CommandMapper, we want to check for equality based
    // on the mapState since we don't want two identical map_states with unique map_commands.
    // Even if mappedCommand vectors are different we want insertion to fail.
    return cm1.mapState == cm2.mapState;
}

bool CommandMapping::mappingSubset(const RemoteMapState &mapState)
{
    return this->mapState.stateSubsetOf(mapState);
}

void CommandMapping::addCommands()
{
    std::for_each(mappedCommands.begin(), mappedCommands.end(), [this](auto cmd) {
        drivers->commandScheduler.addCommand(cmd);
    });
}

void CommandMapping::removeCommands()
{
    std::for_each(mappedCommands.begin(), mappedCommands.end(), [this](auto cmd) {
        drivers->commandScheduler.removeCommand(cmd, false);
    });
}

bool CommandMapping::noCommandsScheduled() const
{
    for (auto cmd : mappedCommands)
    {
        if (drivers->commandScheduler.isCommandScheduled(cmd))
        {
            return false;
        }
    }
    return true;
}
}  // namespace control
}  // namespace tap
