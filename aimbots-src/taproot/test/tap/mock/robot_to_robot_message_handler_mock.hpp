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

#include <gmock/gmock.h>

#include "tap/communication/serial/ref_serial.hpp"

namespace tap::mock
{
class RobotToRobotMessageHandlerMock : public tap::serial::RefSerial::RobotToRobotMessageHandler
{
public:
    RobotToRobotMessageHandlerMock();
    MOCK_METHOD1(functorOp, void(const tap::serial::DJISerial::SerialMessage &));
    void operator()(const tap::serial::DJISerial::SerialMessage &message) override
    {
        return functorOp(message);
    }
};
}  // namespace tap::mock
