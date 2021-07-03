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

#include "uart_terminal_device.hpp"

#include "aruwlib/drivers.hpp"

namespace aruwlib
{
namespace communication
{
namespace serial
{
UartTerminalDevice::UartTerminalDevice(Drivers *drivers) : drivers(drivers) {}

void UartTerminalDevice::initialize() { drivers->uart.init<TERMINAL_UART_PORT, UART_BAUDE_RATE>(); }

bool UartTerminalDevice::read(char &c)
{
    return drivers->uart.read(TERMINAL_UART_PORT, &reinterpret_cast<uint8_t &>(c));
}

void UartTerminalDevice::write(char c) { drivers->uart.write(TERMINAL_UART_PORT, c); }

void UartTerminalDevice::flush() { drivers->uart.flushWriteBuffer(TERMINAL_UART_PORT); }
}  // namespace serial
}  // namespace communication
}  // namespace aruwlib
