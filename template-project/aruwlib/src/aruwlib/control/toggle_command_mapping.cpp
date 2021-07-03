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

#include "toggle_command_mapping.hpp"

namespace aruwlib
{
namespace control
{
void ToggleCommandMapping::executeCommandMapping(const RemoteMapState &currState)
{
    // Neg keys are weird in this mapping and must be handled as such. If neg keys of the
    // map state are a subset of the currState's neg keys, the mapping must be reset
    // and commands removed.
    if (mapState.getNegKeysUsed() && negKeysSubset(mapState, currState))
    {
        if (toggled)
        {
            removeCommands();
        }
        toggled = false;
        pressed = false;
    }
    else if (mappingSubset(currState))
    {
        if (!pressed)
        {
            if (!toggled)
            {
                addCommands();
            }
            else
            {
                removeCommands();
            }
            toggled = !toggled;
            pressed = true;
        }
    }
    else
    {
        pressed = false;
    }
}
}  // namespace control
}  // namespace aruwlib
