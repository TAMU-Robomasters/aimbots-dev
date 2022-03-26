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

#ifndef TAPROOT_UART_HPP_
#define TAPROOT_UART_HPP_

#include <cstdint>
#include <cstdlib>

#ifndef PLATFORM_HOSTED
#include "modm/platform.hpp"
#include "modm/platform/uart/uart_base.hpp"
#endif

#include "tap/board/board.hpp"
#include "tap/util_macros.hpp"

namespace tap::communication::serial
{
/**
 * Class that wraps modm's Uart implementation.
 *
 * Currently only wraps the uart ports that we are generating modm
 * code for. If additional `UartPort`'s are added, they must be added
 * to this wrapper class here.
 */
class Uart
{
public:
    enum UartPort
    {
        Uart1,
        Uart2,
        Uart3,
        Uart6,
        Uart7,
        Uart8,
    };

#ifdef PLATFORM_HOSTED
    enum Parity
    {
        Disabled,
        Even,
        Odd
    };
#else
    using Parity = modm::platform::UartBase::Parity;
#endif

    Uart() = default;
    DISALLOW_COPY_AND_ASSIGN(Uart)
    mockable ~Uart() = default;

    /**
     * Initializes a particular Uart with the pins particular to the board.
     *
     * @note follow covention in the functino when adding a `UartPort`.
     * @tparam port the particular port to initialize.
     * @tparam baudrate desired baud rate in Hz.
     * @tparam parity @see `Parity`.
     */
    template <UartPort port, modm::baudrate_t baudrate, Parity parity = Parity::Disabled>
    void init()
    {
#ifndef PLATFORM_HOSTED
        if constexpr (port == UartPort::Uart1)
        {
            modm::platform::Usart1::connect<GpioB7::Rx>();
            modm::platform::Usart1::initialize<Board::SystemClock, baudrate>(parity);
        }
        else if constexpr (port == UartPort::Uart2)
        {
            modm::platform::Usart2::connect<GpioD5::Tx, GpioD6::Rx>();
            modm::platform::Usart2::initialize<Board::SystemClock, baudrate>(parity);
        }
        else if constexpr (port == UartPort::Uart3)
        {
            modm::platform::Usart3::connect<GpioD8::Tx, GpioD9::Rx>();
            modm::platform::Usart3::initialize<Board::SystemClock, baudrate>(parity);
        }
        else if constexpr (port == UartPort::Uart6)
        {
            modm::platform::Usart6::connect<GpioG14::Tx, GpioG9::Rx>();
            modm::platform::Usart6::initialize<Board::SystemClock, baudrate>(parity);
        }
        else if constexpr (port == UartPort::Uart7)
        {
            modm::platform::Uart7::connect<GpioE8::Tx, GpioE7::Rx>();
            modm::platform::Uart7::initialize<Board::SystemClock, baudrate>(parity);
        }
        else if constexpr (port == UartPort::Uart8)
        {
            modm::platform::Uart8::connect<GpioE1::Tx, GpioE0::Rx>();
            modm::platform::Uart8::initialize<Board::SystemClock, baudrate>(parity);
        }
#endif
    }

    /**
     * Read a single byte.
     *
     * @param[in] port the port to read from.
     * @param[out] data Byte read, if any.
     *
     * @return `true` if a byte was received, `false` otherwise.
     */
    mockable bool read(UartPort port, uint8_t *data);

    /**
     * Read a block of bytes.
     *
     * @param[in] port the port to read from.
     * @param[out] data pointer to a buffer big enough to store `length` bytes
     * @param[in] length number of bytes to be read.
     *
     * @return number of bytes which could be read, maximal `length`.
     */
    mockable std::size_t read(UartPort port, uint8_t *data, std::size_t length);

    /**
     * Empty the receive FIFO queue and hardware buffer.
     *
     * @param[in] port the port's buffer to discard.
     * @return the size of the deleted FIFO queue.
     */
    mockable std::size_t discardReceiveBuffer(UartPort port);

    /**
     * Pushes a single byte into the buffer.
     *
     * @param[in] port the port to write to.
     * @return `true` if data has been successfully sent, `false` if buffer is full.
     * @note this writing is buffered.
     */
    mockable bool write(UartPort port, uint8_t data);

    /**
     * Pushes a block of bytes into the buffer.
     *
     * @param[in] port the port to write to.
     * @param[in] data pointer to a buffer big enough to store `length` bytes.
     * @param[in] length number of bytes to be written.
     * @return the number of bytes that have been written.
     * @note this writing may be buffered.
     */
    mockable std::size_t write(UartPort port, const uint8_t *data, std::size_t length);

    /**
     * Because the data is buffered, check here to see if the buffer is empty
     * (implying everything has been written).
     *
     * @param[in] port the port to see if writing is finished.
     * @return `true` if the buffer is empty and the last byte has been sent.
     */
    mockable bool isWriteFinished(UartPort port) const;

    mockable void flushWriteBuffer(UartPort port);
};

}  // namespace tap::communication::serial

#endif  // TAPROOT_UART_HPP_
