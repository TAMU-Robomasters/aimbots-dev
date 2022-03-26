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

#ifndef TAPROOT_COMMAND_MAPPING_HPP_
#define TAPROOT_COMMAND_MAPPING_HPP_

#include <vector>

#include "remote_map_state.hpp"

namespace tap
{
namespace control
{
class Command;
/**
 * A class that combines a vector of `Command`s and a RemoteMapState whose behavior
 * is defined by derived classes. Used in conjunction with the CommandMapper to
 * add and remove `Command`s from the scheduler when the derived CommandMapping
 * sees fit, depending on the current state of the remote data.
 *
 * @see HoldCommandMapping
 * @see HoldRepeatCommandMapping
 * @see PressCommandMapping
 * @see ToggleCommandMapping
 */
class CommandMapping
{
public:
    /**
     * Initializes the CommandMapping with the set of passed in `Command`s mapped to
     * a particular RemoteMapState.
     *
     * @note All nullptr `Command`s in cmds will be removed.
     * @param[in] cmds A list of `Command`s that are associated with the mapping.
     * @param[in] rms The map state that will be compared to the actual remote state
     *      to determine whether or not to add `cmds`.
     */
    CommandMapping(Drivers *drivers, const std::vector<Command *> cmds, const RemoteMapState &rms);

    DISALLOW_COPY_AND_ASSIGN(CommandMapping)

    /**
     * Straight equality of the mapState and mappedCommands between cm1 and cm2.
     */
    friend bool operator==(const CommandMapping &cm1, const CommandMapping &cm2);

    /**
     * Checks for equality between the `mapState`s of cm1 and cm2.
     */
    friend bool mapStateEqual(const CommandMapping &cm1, const CommandMapping &cm2);

    /**
     * Nothing dynamically allocated that isn't taken care of automatically.
     */
    virtual ~CommandMapping() = default;

    /**
     * Using currState, determines whether or not to add or remove `Command`s from
     * the main scheduler. Up the the implementer to determine what the criteria
     * for adding and removing `Command`s should be.
     *
     * @param[in] currState The current state of the remote.
     */
    virtual void executeCommandMapping(const RemoteMapState &currState) = 0;

    /**
     * @return `true` if `this`'s `mapState` is a subset of the passed in
     *      `mapState`. Returns `false` otherwise.
     */
    virtual bool mappingSubset(const RemoteMapState &mapState);

    /**
     * @return `true` if `state1`'s neg keys are a subset of `state2`'s keys pressed, `false`
     *      otherwise.
     */
    static inline bool negKeysSubset(const RemoteMapState &state1, const RemoteMapState &state2)
    {
        return state1.getNegKeys() == (state1.getNegKeys() & state2.getKeys());
    }

    const RemoteMapState &getAssociatedRemoteMapState() const { return mapState; }

    const std::vector<Command *> &getAssociatedCommands() const { return mappedCommands; }

protected:
    /**
     * The RemoteMapState specified when constructing the CommandMapping.
     */
    const RemoteMapState mapState;

    /**
     * A map of commands to add to and remove from the scheduler.
     */
    std::vector<Command *> mappedCommands;

    Drivers *drivers;

    /**
     * Adds all the `Command`s to the main CommandScheduler.
     */
    void addCommands();

    /**
     * Removes all the `Command`s from the main CommandScheduler.
     */
    void removeCommands();

    /**
     * @return True if none of the associated commands are scheduled.
     */
    bool noCommandsScheduled() const;
};  // class CommandMapping
}  // namespace control
}  // namespace tap

#endif  // TAPROOT_COMMAND_MAPPING_HPP_
