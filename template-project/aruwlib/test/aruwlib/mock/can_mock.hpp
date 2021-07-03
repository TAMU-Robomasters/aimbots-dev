#ifndef MOCK_CAN_HPP_
#define MOCK_CAN_HPP_

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

#include <gmock/gmock.h>

#include "aruwlib/communication/can/can.hpp"

#include "modm/architecture/interface/can_message.hpp"

namespace aruwlib
{
namespace mock
{
class CanMock : public aruwlib::can::Can
{
public:
    CanMock();
    virtual ~CanMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(bool, isMessageAvailable, (aruwlib::can::CanBus bus), (const override));
    MOCK_METHOD(
        bool,
        getMessage,
        (aruwlib::can::CanBus bus, modm::can::Message *message),
        (override));
    MOCK_METHOD(bool, isReadyToSend, (aruwlib::can::CanBus bus), (const override));
    MOCK_METHOD(
        bool,
        sendMessage,
        (aruwlib::can::CanBus bus, const modm::can::Message &message),
        (override));
};  // class CanMock
}  // namespace mock
}  // namespace aruwlib

#endif  // MOCK_CAN_HPP_
