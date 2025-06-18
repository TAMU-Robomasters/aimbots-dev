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

#ifndef TAPROOT_UART_TERMINAL_DEVICE_HPP_
#define TAPROOT_UART_TERMINAL_DEVICE_HPP_

#include "tap/communication/serial/uart.hpp"

#include "modm/io/iodevice.hpp"

#include "uart_terminal_device_constants.hpp"

namespace tap
{
class Drivers;
}

namespace tap::communication::serial
{
/**
 * A wrapper around UART3 used by the terminal handler. Allows for stream-based operations
 * to be performed on data being sent/received by the UART line by using an instance
 * of this class with a `modm::IOStream`.
 */
class UartTerminalDevice : public ::modm::IODevice
{
public:
    UartTerminalDevice(Drivers *drivers);
    DISALLOW_COPY_AND_ASSIGN(UartTerminalDevice);
    virtual ~UartTerminalDevice() = default;

    void initialize();

    /**
     * Reads a byte from the UART receive buffer and populates `c` with the value.
     *
     * @param[out] c The byte that data will be read into.
     */
    bool read(char &c) override;

    using IODevice::write;
    /**
     * Writes the character `c` into the UART buffer.
     *
     * @param[out] c The byte to write to the buffer.
     */
    void write(char c) override;

    /**
     * Flushes the UART tx buffer.
     */
    void flush() override;

private:
    static constexpr uint32_t UART_BAUDE_RATE = 115200;

    Drivers *drivers;

    static constexpr Uart::UartPort TERMINAL_UART_PORT = bound_ports::TERMINAL_SERIAL_UART_PORT;
};  // class UartTerminalDevice
}  // namespace tap::communication::serial

#endif  // TAPROOT_UART_TERMINAL_DEVICE_HPP_
