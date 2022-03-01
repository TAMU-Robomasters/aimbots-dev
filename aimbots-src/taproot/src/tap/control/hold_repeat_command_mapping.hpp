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

#ifndef HOLD_REPEAT_MAPPING_HPP_
#define HOLD_REPEAT_MAPPING_HPP_

#include "command_mapping.hpp"

namespace tap
{
namespace control
{
class Command;
class RemoteMapState;

/**
 * A CommandMapping that adds `Command`s when the contained mapping is a subset of the remote
 * mapping. If a Command finishes and the contained mapping is still a subset of the remote mapping,
 * it is added again. It then removes the `Command`s when the mapping is no longer a subset if
 * endCommandsWhenNotHeld is true or doesn't end commands if endCommandsWhenNotHeld is false.
 *
 * Additionally, When neg keys are being used and the mapping's neg keys are a subset of the remote
 * map state, the `Command`s are removed.
 */
class HoldRepeatCommandMapping : public CommandMapping
{
public:
    /**
     * Constructor must take the set of `Command`s and the RemoteMapState.
     *
     * @param[in] drivers Global drivers instance.
     * @param[in] cmds vector of commands that will be scheduled by this command mapping.
     * @param[in] rms RemoteMapState that controls when commands will be scheduled.
     * @param[in] endCommandsWhenNotHeld If `true`, the commands will be forcibly ended by the
     * command mapping when no longer being held. Otherwise, the commands will naturally finish.
     */
    HoldRepeatCommandMapping(
        Drivers *drivers,
        const std::vector<Command *> cmds,
        const RemoteMapState &rms,
        bool endCommandsWhenNotHeld)
        : CommandMapping(drivers, cmds, rms),
          commandsScheduled(false),
          endCommandsWhenNotHeld(endCommandsWhenNotHeld)
    {
    }

    /**
     * Default destructor.
     */
    ~HoldRepeatCommandMapping() override = default;

    void executeCommandMapping(const RemoteMapState &currState) override;

private:
    bool commandsScheduled;
    bool endCommandsWhenNotHeld;
};  // class HoldRepeatCommandMapping
}  // namespace control
}  // namespace tap

#endif  // HOLD_REPEAT_COMMAND_MAPPING_HPP_
