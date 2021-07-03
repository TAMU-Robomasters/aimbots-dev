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

#ifndef REMOTE_MOCK_HPP_
#define REMOTE_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwlib/communication/remote.hpp"

namespace aruwlib
{
namespace mock
{
class RemoteMock : public aruwlib::Remote
{
public:
    RemoteMock(aruwlib::Drivers *drivers);
    virtual ~RemoteMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, read, (), (override));
    MOCK_METHOD(bool, isConnected, (), (const override));
    MOCK_METHOD(float, getChannel, (aruwlib::Remote::Channel ch), (const override));
    MOCK_METHOD(
        aruwlib::Remote::SwitchState,
        getSwitch,
        (aruwlib::Remote::Switch sw),
        (const override));
    MOCK_METHOD(int16_t, getMouseX, (), (const override));
    MOCK_METHOD(int16_t, getMouseY, (), (const override));
    MOCK_METHOD(int16_t, getMouseZ, (), (const override));
    MOCK_METHOD(bool, getMouseL, (), (const override));
    MOCK_METHOD(bool, getMouseR, (), (const override));
    MOCK_METHOD(bool, keyPressed, (aruwlib::Remote::Key key), (const override));
    MOCK_METHOD(int16_t, getWheel, (), (const override));
    MOCK_METHOD(uint32_t, getUpdateCounter, (), (const override));
};  // class RemoteMock
}  // namespace mock
}  // namespace aruwlib

#endif  // REMOTE_MOCK_HPP_
