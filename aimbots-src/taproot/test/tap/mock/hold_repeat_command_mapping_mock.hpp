/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_HOLD_REPEAT_COMMAND_MAPPING_MOCK_HPP_
#define TAPROOT_HOLD_REPEAT_COMMAND_MAPPING_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/control/hold_repeat_command_mapping.hpp"

namespace tap::mock
{
class HoldRepeatCommandMappingMock : public control::HoldRepeatCommandMapping
{
public:
    HoldRepeatCommandMappingMock(
        Drivers *drivers,
        const std::vector<control::Command *> cmds,
        const control::RemoteMapState &rms,
        bool endCommandsWhenNotHeld,
        int maxTimesToSchedule = -1);
    virtual ~HoldRepeatCommandMappingMock();

    MOCK_METHOD(void, executeCommandMapping, (const tap::control::RemoteMapState &), (override));
    MOCK_METHOD(void, setMaxTimesToSchedule, (int), (override));
};  // class HoldRepeatCommandMappingMock
}  // namespace tap::mock

#endif  // TAPROOT_HOLD_REPEAT_COMMAND_MAPPING_MOCK_HPP_
