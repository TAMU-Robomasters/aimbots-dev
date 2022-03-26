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

#ifndef TAPROOT_PRESS_COMMAND_MAPPING_HPP_
#define TAPROOT_PRESS_COMMAND_MAPPING_HPP_

#include "command_mapping.hpp"

namespace tap
{
namespace control
{
class Command;
class RemoteMapState;

/**
 * A CommandMapping that adds `Command`s when the contained
 * mapping is a subset of the remote mapping. The Command is not removed
 * by the CommandMapping. Instead, the mapping will be removed when the
 * Command is finished.
 */
class PressCommandMapping : public CommandMapping
{
public:
    /**
     * Constructor must take the set of `Command`s and the RemoteMapState.
     */
    PressCommandMapping(
        Drivers *drivers,
        const std::vector<Command *> cmds,
        const RemoteMapState &rms)
        : CommandMapping(drivers, cmds, rms),
          pressed(false)
    {
    }

    /**
     * Default destructor.
     */
    ~PressCommandMapping() override = default;

    void executeCommandMapping(const RemoteMapState &currState) override;

private:
    bool pressed;
};  // class PressCommandMapping
}  // namespace control
}  // namespace tap

#endif  // TAPROOT_PRESS_COMMAND_MAPPING_HPP_
