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

#ifndef TAPROOT_UART_MOCK_HPP_
#define TAPROOT_UART_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/serial/uart.hpp"

namespace tap
{
namespace mock
{
class UartMock : public tap::communication::serial::Uart
{
public:
    UartMock();
    virtual ~UartMock();

    MOCK_METHOD(
        bool,
        read,
        (tap::communication::serial::Uart::UartPort port, uint8_t *data),
        (override));
    MOCK_METHOD(
        std::size_t,
        read,
        (tap::communication::serial::Uart::UartPort port, uint8_t *data, std::size_t length),
        (override));
    MOCK_METHOD(
        std::size_t,
        discardReceiveBuffer,
        (tap::communication::serial::Uart::UartPort port),
        (override));
    MOCK_METHOD(
        bool,
        write,
        (tap::communication::serial::Uart::UartPort port, uint8_t data),
        (override));
    MOCK_METHOD(
        std::size_t,
        write,
        (tap::communication::serial::Uart::UartPort port, const uint8_t *data, std::size_t length),
        (override));
    MOCK_METHOD(
        bool,
        isWriteFinished,
        (tap::communication::serial::Uart::UartPort port),
        (const override));
    MOCK_METHOD(
        void,
        flushWriteBuffer,
        (tap::communication::serial::Uart::UartPort port),
        (override));
};  // class UartMock
}  // namespace mock
}  // namespace tap

#endif  // TAPROOT_UART_MOCK_HPP_
