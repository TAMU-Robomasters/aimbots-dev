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

#ifndef CAN_RX_LISTENER_MOCK_HPP_
#define CAN_RX_LISTENER_MOCK_HPP_

#include <iostream>

#include <gmock/gmock.h>

#include "aruwlib/communication/can/can_rx_listener.hpp"

#include "modm/architecture/interface/can_message.hpp"

namespace aruwlib
{
namespace mock
{
class CanRxListenerMock : public aruwlib::can::CanRxListener
{
public:
    CanRxListenerMock(aruwlib::Drivers* drivers, uint32_t id, aruwlib::can::CanBus bus);
    ~CanRxListenerMock();

    MOCK_METHOD(void, processMessage, (const modm::can::Message& message), (override));

};  // class CanRxListenerMock
}  // namespace mock
}  // namespace aruwlib

#endif  //  CAN_RX_LISTENER_MOCK_HPP_
