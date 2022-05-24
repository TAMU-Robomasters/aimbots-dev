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

#ifndef TAPROOT_TERMINAL_SERIAL_MOCK_HPP_
#define TAPROOT_TERMINAL_SERIAL_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/serial/terminal_serial.hpp"

namespace tap
{
namespace mock
{
class TerminalSerialMock : public communication::serial::TerminalSerial
{
public:
    TerminalSerialMock(Drivers *drivers);
    virtual ~TerminalSerialMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, update, (), (override));
    MOCK_METHOD(
        void,
        addHeader,
        (const char *, communication::serial::TerminalSerialCallbackInterface *),
        (override));
};  // class TerminalSerialMock
}  // namespace mock
}  // namespace tap

#endif  // TAPROOT_TERMINAL_SERIAL_MOCK_HPP_
