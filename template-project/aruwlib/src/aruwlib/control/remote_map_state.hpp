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

#ifndef REMOTE_MAP_STATE_HPP_
#define REMOTE_MAP_STATE_HPP_

#include <cstdint>
#include <list>

#include "aruwlib/communication/remote.hpp"

namespace aruwlib
{
namespace control
{
/**
 * A class to be used in conjunction with a CommandMapping to be placed in
 * the CommandMapper. A particular RemoteMapState is used to capture a sequence
 * of user inputs that must be triggered for a Command to be scheduled.
 *
 * To use this class, when possible use one of the constructors provided below
 * to create a mapping. We have the ability to map any combination of remote
 * switches, keyboard keys, or mouse buttons to a particular set of `Command`s.
 * To cover every possible combination, additional initialize functions have
 * been provided to be used in conjunction with class constructors.
 *
 * @see CommandMapper for information about adding a `RemoteMapState` to the
 *      CommandMapper.
 *
 * @note <b>What is a "neg key"?</b> I frequently will refer to a `negKeySet`.
 *      This can be thought of a key mapping that when matched, no matter what
 *      the state of the RemoteMapState is, the RemoteMapState is no longer
 *      satisfied.
 */
class RemoteMapState
{
public:
    /**
     * Use to distinguish between the left and mouse button when initializing
     * a RemoteMapState that uses the mouse as input.
     */
    enum class MouseButton
    {
        LEFT,  /// The left mouse button.
        RIGHT  /// The right mouse button.
    };

    RemoteMapState() = default;

    /**
     * Initializes a RemoteMapState with a single switch to the given switch state.
     *
     * @param[in] swh The switch to use in the map state.
     * @param[in] switchState The switch state of the given switch.
     */
    RemoteMapState(Remote::Switch swh, Remote::SwitchState switchState);

    /**
     * Initializes a RemoteMapState with particular switch states for both remote
     * switches.
     *
     * @param[in] leftss The switch state for the left switch.
     * @param[in] rightss The switch state for the right switch.
     */
    RemoteMapState(Remote::SwitchState leftss, Remote::SwitchState rightss);

    /**
     * Initializes a RemoteMapState with a particular set of keys and optionally a
     * particular set of negation keys.
     *
     * @param[in] keySet The set of keys to use for initialization.
     * @param[in] negKeySet The set of keys to be used as negations in the RemoteMapState.
     * @note `keySet` and `negKeySet` must be mutally exclusive sets, otherwise the
     *      `negKeySet` will not be properly initialized.
     */
    RemoteMapState(
        const std::list<Remote::Key> &keySet,
        const std::list<Remote::Key> &negKeySet = {});

    /**
     * Initializes a RemoteMapState with a particular mouse button and set of keys and
     * optionally a particular set of negation keys.
     *
     * @param[in] button The mouse button The button to use for initialization.
     * @param[in] keySet The set of keys to use for initialization.
     * @param[in] negKeySet The set of keys to be used as negations in the RemoteMapState.
     * @note `keySet` and `negKeySet` must be mutally exclusive sets, otherwise the
     *      `negKeySet` will not be properly initialized.
     */
    RemoteMapState(
        RemoteMapState::MouseButton button,
        const std::list<Remote::Key> &keySet,
        const std::list<Remote::Key> &negKeySet = {});

    /**
     * Initializes a RemoteMapState that will use the given mouse button (either left or
     * right) in the mapping.
     *
     * @param[in] button The MouseButton to use.
     */
    RemoteMapState(MouseButton button);

    /**
     * Initializes the left switch with the particular `Remote::SwitchState` provided.
     */
    void initLSwitch(Remote::SwitchState ss);

    /**
     * Initializes the right switch with the particular `Remote::SwitchState` provided.
     */
    void initRSwitch(Remote::SwitchState ss);

    /**
     * Initializes the keys to the bit mapped set of keys provided.
     * @note `keys` must be mutally exclusive with any set of `negKeys` already provided.
     */
    void initKeys(uint16_t keys);

    /**
     * Initializes the neg keys to the bit mapped set of neg keys provided.
     * @note `negKeys` must be mutally exclusive with any set of `keys` already provided.
     */
    void initNegKeys(uint16_t negKeys);

    /**
     * @see `initKeys`. Interprets the list and passes that on as a bit mapped set of keys.
     */
    void initKeys(const std::list<Remote::Key> &keySet);

    /**
     * @see `initNegKeys`. Interprets the list and passes that on as a bit mapped set of keys.
     */
    void initNegKeys(const std::list<Remote::Key> &negKeySet);

    /**
     * Initializes the left mouse button to be mapped when clicked.
     */
    void initLMouseButton();

    /**
     * Initializes the right mouse button to be mapped when clicked.
     */
    void initRMouseButton();

    /**
     * Checks if `this` is a subset of `other`. `this` is a subset of `other` under the following
     * conditions:
     * - Either `this`'s left switch state is `UNKNOWN` or `this`'s left switch state is equal to
     *   `other`'s left switch state.
     * - Either `this`'s right switch state is `UNKNOWN` or `this`'s right switch state is equal to
     *   `other`'s left switch state.
     * - Either `this`'s left mouse button is not initialized or both `this` and `other`'s left
     *   mouse buttons are both initialized.
     * - Either `this`'s right mouse button is not initialized or both `this` and `other`'s right
     *   mouse buttons are both initialized.
     * - `this`'s key set is a subset of `other`'s key set, i.e. `(this.keySet & other.keySet) ==
     *   this.keySet`.
     *
     * @attention This function does not use neg keys to determine if the map
     *      state is a subset.
     *
     * @param[other] The RemoteMapState to check if `this` is a subset of.
     * @return `true` if `this` RemoteMapState is a subset of the `other` RemoteMapState. See above
     * for description of what it means for a `RemoteMapState` to be a subset of another.
     */
    bool stateSubsetOf(const RemoteMapState &other) const;

    /**
     * Straight equality.
     *
     * @param[in] rms1 The first RemoteMapState to check equality for.
     * @param[in] rms1 The second RemoteMapState to check equality for.
     */
    bool friend operator==(const RemoteMapState &rms1, const RemoteMapState &rms2);

    /**
     * Opposite of operator==.
     */
    bool friend operator!=(const RemoteMapState &rms1, const RemoteMapState &rms2);

    /**
     * @return The negKeys currently being used.
     */
    uint16_t getNegKeys() const { return negKeys; }

    /**
     * @return `true` if the neg key set has been initialized, `false` otherwise.
     */
    bool getNegKeysUsed() const { return negKeys != 0; }

    /**
     * @return the current keys initialized in the `RemoteMapState`.
     */
    uint16_t getKeys() const { return keys; }

    bool getLMouseButton() const { return lMouseButton; }

    bool getRMouseButton() const { return rMouseButton; }

    Remote::SwitchState getLSwitch() const { return lSwitch; }

    Remote::SwitchState getRSwitch() const { return rSwitch; }

private:
    Remote::SwitchState lSwitch = Remote::SwitchState::UNKNOWN;

    Remote::SwitchState rSwitch = Remote::SwitchState::UNKNOWN;

    uint16_t keys = 0;

    uint16_t negKeys = 0;  // if certain keys are pressed, the remote map will not do mapping

    bool lMouseButton = false;

    bool rMouseButton = false;
};  // class RemoteState
}  // namespace control
}  // namespace aruwlib

#endif  // REMOTE_MAP_STATE_HPP_
