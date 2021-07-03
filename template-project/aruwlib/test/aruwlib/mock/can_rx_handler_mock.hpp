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

#ifndef CAN_RX_HANDLER_MOCK_HPP_
#define CAN_RX_HANDLER_MOCK_HPP_

#include <iostream>

#include <gmock/gmock.h>

#include "aruwlib/communication/can/can_rx_handler.hpp"

namespace aruwlib
{
namespace mock
{
class CanRxHandlerMock : public aruwlib::can::CanRxHandler
{
public:
    CanRxHandlerMock(aruwlib::Drivers* drivers);
    virtual ~CanRxHandlerMock();

    MOCK_METHOD(
        void,
        attachReceiveHandler,
        (aruwlib::can::CanRxListener* const listener),
        (override));
    MOCK_METHOD(void, pollCanData, (), (override));
    MOCK_METHOD(
        void,
        removeReceiveHandler,
        (const aruwlib::can::CanRxListener& rxListener),
        (override));
};  // class CanRxHandlerMock
}  // namespace mock
}  // namespace aruwlib

#endif  //  CAN_RX_HANDLER_MOCK_HPP_
