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

#ifndef TAPROOT_REMOTE_MOCK_HPP_
#define TAPROOT_REMOTE_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/serial/remote.hpp"

namespace tap
{
namespace mock
{
class RemoteMock : public tap::communication::serial::Remote
{
public:
    RemoteMock(tap::Drivers *drivers);
    virtual ~RemoteMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, read, (), (override));
    MOCK_METHOD(bool, isConnected, (), (const override));
    MOCK_METHOD(
        float,
        getChannel,
        (tap::communication::serial::Remote::Channel ch),
        (const override));
    MOCK_METHOD(
        tap::communication::serial::Remote::SwitchState,
        getSwitch,
        (tap::communication::serial::Remote::Switch sw),
        (const override));
    MOCK_METHOD(int16_t, getMouseX, (), (const override));
    MOCK_METHOD(int16_t, getMouseY, (), (const override));
    MOCK_METHOD(int16_t, getMouseZ, (), (const override));
    MOCK_METHOD(bool, getMouseL, (), (const override));
    MOCK_METHOD(bool, getMouseR, (), (const override));
    MOCK_METHOD(bool, keyPressed, (tap::communication::serial::Remote::Key key), (const override));
    MOCK_METHOD(int16_t, getWheel, (), (const override));
    MOCK_METHOD(uint32_t, getUpdateCounter, (), (const override));
};  // class RemoteMock
}  // namespace mock
}  // namespace tap

#endif  // TAPROOT_REMOTE_MOCK_HPP_
