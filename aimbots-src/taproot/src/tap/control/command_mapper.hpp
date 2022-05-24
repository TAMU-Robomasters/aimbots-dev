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

#ifndef TAPROOT_COMMAND_MAPPER_HPP_
#define TAPROOT_COMMAND_MAPPER_HPP_

#include <vector>

#include "tap/communication/serial/remote.hpp"
#include "tap/util_macros.hpp"

namespace tap
{
class Drivers;
namespace control
{
class CommandMapping;

/**
 * Class that controls mapping remote state to actions. All the remote
 * mappings will be handled here. One passes a RemoteMapState and a set
 * of `Command`s for which the RemoteMapState is mapped to to one of
 * the `add<type>Mapping` functions. This will create an appropriate CommandMapping
 * and add it to the list of mappings to be checked each time new remote information
 * is received.
 *
 * For example, given the command `coolCommand`, to map a hold mapping
 * to the left switch in the up position, we define a `HoldCommandMapping`
 * and then add it to the `CommandMapper` as follows (where `drivers` is assumed
 * to be a global drivers object that has a `CommandMapper` object):
 *
 * ```
 * HoldCommandMapping leftSwitchUp(
 *      drivers,
 *      {&coolCommand},
 *      RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
 *
 * drivers->commandMapper.addMap(&leftSwitchUp);
 * ```
 *
 * @note Only unique RemoteMapStates can be added to the CommandMapper. This ensures
 *      a user will not accidently map two `Command`s to the same RemoteMapState without
 *      knowing they did so. Instead, the user must explicitly add `Command`s to a common
 *      vector that maps to a single RemoteMapState.
 */
class CommandMapper
{
public:
    explicit CommandMapper(Drivers *drivers) : drivers(drivers) {}
    DISALLOW_COPY_AND_ASSIGN(CommandMapper)
    mockable ~CommandMapper() = default;

    /**
     * The heart of the CommandMapper.
     *
     * Iterates through all the current mappings to see which buttons are pressed
     * in order to determine which commands should be added to or removed from the scheduler.
     * Call when new remote information has been received.
     */
    mockable void handleKeyStateChange(
        uint16_t key,
        tap::communication::serial::Remote::SwitchState leftSwitch,
        tap::communication::serial::Remote::SwitchState rightSwitch,
        bool mouseL,
        bool mouseR);

    /**
     * Verifies the mapping passed in can be added to `commandsToRun`
     * and if possible adds the mapping.
     *
     * @param[in] mapping A pointer to the CommandMapping to be added. The
     *      command mapper is not responsible for memory deallocation of this
     *      command mapping.
     */
    mockable void addMap(CommandMapping *mapping);

    /**
     * @return the number of command mappings in the mapper.
     */
    mockable std::size_t getSize() const { return commandsToRun.size(); }

    /**
     * @return The CommandMapping located at the specificed index, or
     *      `nullptr` of the index is out of bounds.
     */
    mockable const CommandMapping *getAtIndex(std::size_t index) const;

private:
    /**
     * We use a vector because it is slightly faster for iteration than an `std::set` or
     * `std::map` (which would facilitate a different structure than a `CommandMapping` class).
     * While inserting, we ensure CommandMappings with identical RemoteMapStates fail to be added.
     * It ends up being slower to insert, but this is OK since we only insert at the beginning
     * of execution.
     */
    std::vector<CommandMapping *> commandsToRun;

    Drivers *drivers;
};  // class CommandMapper

}  // namespace control
}  // namespace tap

#endif  // TAPROOT_COMMAND_MAPPER_HPP_
