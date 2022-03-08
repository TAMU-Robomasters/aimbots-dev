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

#ifndef MPU6500_TERMINAL_SERIAL_HANDLER_MOCK_HPP_
#define MPU6500_TERMINAL_SERIAL_HANDLER_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/sensors/mpu6500/mpu6500_terminal_serial_handler.hpp"

namespace tap
{
namespace mock
{
class Mpu6500TerminalSerialHandlerMock : public tap::sensors::Mpu6500TerminalSerialHandler
{
public:
    Mpu6500TerminalSerialHandlerMock(tap::Drivers *drivers);
    virtual ~Mpu6500TerminalSerialHandlerMock();

    MOCK_METHOD(void, init, (), (override));
};  // Mpu6500TerminalSerialHandlerMock
}  // namespace mock
}  // namespace tap

#endif  //  MPU6500_TERMINAL_SERIAL_HANDLER_MOCK_HPP_
