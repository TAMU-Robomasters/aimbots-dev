/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "remote.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "remote_serial_constants.hpp"

namespace tap::communication::serial
{
void Remote::initialize()
{
    drivers->uart.init<bound_ports::REMOTE_SERIAL_UART_PORT, 100000, Uart::Parity::Even>();
}

void Remote::read()
{
    // Check disconnect timeout
    if (tap::arch::clock::getTimeMilliseconds() - lastRead > REMOTE_DISCONNECT_TIMEOUT)
    {
        connected = false;  // Remote no longer connected
        reset();            // Reset current remote values
    }
    uint8_t data;  // Next byte to be read
    // Read next byte if available and more needed for the current packet
    while (drivers->uart.read(bound_ports::REMOTE_SERIAL_UART_PORT, &data) &&
           currentBufferIndex < REMOTE_BUF_LEN)
    {
        rxBuffer[currentBufferIndex] = data;
        currentBufferIndex++;
        lastRead = tap::arch::clock::getTimeMilliseconds();
    }
    // Check read timeout
    if (tap::arch::clock::getTimeMilliseconds() - lastRead > REMOTE_READ_TIMEOUT)
    {
        clearRxBuffer();
    }
    // Parse buffer if all 18 bytes are read
    if (currentBufferIndex >= REMOTE_BUF_LEN)
    {
        connected = true;
        parseBuffer();
        clearRxBuffer();
    }
}

bool Remote::isConnected() const { return connected; }

float Remote::getChannel(Channel ch) const
{
    switch (ch)
    {
        case Channel::RIGHT_HORIZONTAL:
            return remote.rightHorizontal / STICK_MAX_VALUE;
        case Channel::RIGHT_VERTICAL:
            return remote.rightVertical / STICK_MAX_VALUE;
        case Channel::LEFT_HORIZONTAL:
            return remote.leftHorizontal / STICK_MAX_VALUE;
        case Channel::LEFT_VERTICAL:
            return remote.leftVertical / STICK_MAX_VALUE;
    }
    return 0;
}

Remote::SwitchState Remote::getSwitch(Switch sw) const
{
    switch (sw)
    {
        case Switch::LEFT_SWITCH:
            return remote.leftSwitch;
        case Switch::RIGHT_SWITCH:
            return remote.rightSwitch;
    }
    return SwitchState::UNKNOWN;
}

void Remote::parseBuffer()
{
    // this is a wonky encoding implemented by our "good pal Li Qingzhi" as stated by a chinese
    // translated document our team acquired a while back; refer to this document for the protocol
    // encoding: https://drive.google.com/file/d/1a5kaTsDvG89KQwy3fkLVkxKaQJfJCsnu/view?usp=sharing

    // remote joystick information
    remote.rightHorizontal = (rxBuffer[0] | rxBuffer[1] << 8) & 0x07FF;
    remote.rightHorizontal -= 1024;
    remote.rightVertical = (rxBuffer[1] >> 3 | rxBuffer[2] << 5) & 0x07FF;
    remote.rightVertical -= 1024;
    remote.leftHorizontal = (rxBuffer[2] >> 6 | rxBuffer[3] << 2 | rxBuffer[4] << 10) & 0x07FF;
    remote.leftHorizontal -= 1024;
    remote.leftVertical = (rxBuffer[4] >> 1 | rxBuffer[5] << 7) & 0x07FF;
    remote.leftVertical -= 1024;

    remote.leftSwitch = static_cast<Remote::SwitchState>((rxBuffer[5] >> 6) & 0x03);
    remote.rightSwitch = static_cast<Remote::SwitchState>((rxBuffer[5] >> 4) & 0x03);

    // mouse input
    remote.mouse.x = rxBuffer[6] | (rxBuffer[7] << 8);    // x axis
    remote.mouse.y = rxBuffer[8] | (rxBuffer[9] << 8);    // y axis
    remote.mouse.z = rxBuffer[10] | (rxBuffer[11] << 8);  // z axis
    remote.mouse.l = static_cast<bool>(rxBuffer[12]);     // left button click
    remote.mouse.r = static_cast<bool>(rxBuffer[13]);     // right button click

    // keyboard capture
    remote.key = rxBuffer[14] | rxBuffer[15] << 8;
    // remote wheel
    remote.wheel = (rxBuffer[16] | rxBuffer[17] << 8) - 1024;

    // the remote joystick and wheel values must be <= abs(660)
    if ((abs(remote.rightHorizontal) > STICK_MAX_VALUE) ||
        (abs(remote.rightVertical) > STICK_MAX_VALUE) ||
        (abs(remote.leftHorizontal) > STICK_MAX_VALUE) ||
        (abs(remote.leftVertical) > STICK_MAX_VALUE) || (abs(remote.wheel) > STICK_MAX_VALUE))
    {
        RAISE_ERROR(drivers, "invalid remote joystick values");
    }

    drivers->commandMapper.handleKeyStateChange(
        remote.key,
        remote.leftSwitch,
        remote.rightSwitch,
        remote.mouse.l,
        remote.mouse.r);

    remote.updateCounter++;
}

void Remote::clearRxBuffer()
{
    // Reset bytes read counter
    currentBufferIndex = 0;
    // Clear remote rxBuffer
    for (int i = 0; i < REMOTE_BUF_LEN; i++)
    {
        rxBuffer[i] = 0;
    }
    // Clear Usart1 rxBuffer
    drivers->uart.discardReceiveBuffer(bound_ports::REMOTE_SERIAL_UART_PORT);
}

void Remote::reset()
{
    remote.rightHorizontal = 0;
    remote.rightVertical = 0;
    remote.leftHorizontal = 0;
    remote.leftVertical = 0;
    remote.leftSwitch = SwitchState::UNKNOWN;
    remote.rightSwitch = SwitchState::UNKNOWN;
    remote.mouse.x = 0;
    remote.mouse.y = 0;
    remote.mouse.z = 0;
    remote.mouse.l = 0;
    remote.mouse.r = 0;
    remote.key = 0;
    remote.wheel = 0;
    clearRxBuffer();

    // Refresh command mapper with all keys deactivated. This prevents bug where
    // command states enter defaults when remote reconnects even if key/switch
    // state should do otherwise
    drivers->commandMapper
        .handleKeyStateChange(0, SwitchState::UNKNOWN, SwitchState::UNKNOWN, false, false);
}

uint32_t Remote::getUpdateCounter() const { return remote.updateCounter; }

}  // namespace tap::communication::serial
