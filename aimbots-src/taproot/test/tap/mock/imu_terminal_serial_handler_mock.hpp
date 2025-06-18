/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_IMU_TERMINAL_SERIAL_HANDLER_MOCK_HPP_
#define TAPROOT_IMU_TERMINAL_SERIAL_HANDLER_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"

namespace tap::mock
{
class ImuTerminalSerialHandlerMock : public communication::sensors::imu::ImuTerminalSerialHandler
{
public:
    ImuTerminalSerialHandlerMock(
        tap::Drivers* drivers,
        communication::sensors::imu::ImuInterface* imu);
    virtual ~ImuTerminalSerialHandlerMock();

    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(bool, terminalSerialCallback, (char*, modm::IOStream&, bool), (override));
    MOCK_METHOD(void, terminalSerialStreamCallback, (modm::IOStream&), (override));
};
}  // namespace tap::mock

#endif  // TAPROOT_IMU_TERMINAL_SERIAL_HANDLER_MOCK_HPP_
