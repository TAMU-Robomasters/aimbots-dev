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

#include "tap/architecture/clock.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "dji_serial.hpp"

/**
 * Macro that wraps uart read for ease of readability in code.
 * @param[in] data Byte array where the data should be read into.
 * @param[in] length The number of bytes to read.
 * @return The number of bytes read into data.
 */
#define READ(data, length) drivers->uart.read(this->port, data, length)

namespace tap::communication::serial
{
DJISerial::DJISerial(Drivers *drivers, Uart::UartPort port, bool isRxCRCEnforcementEnabled)
    : port(port),
      djiSerialRxState(SERIAL_HEADER_SEARCH),
      newMessage(),
      mostRecentMessage(),
      frameCurrReadByte(0),
      rxCrcEnabled(isRxCRCEnforcementEnabled),
      drivers(drivers)
{
}

void DJISerial::initialize()
{
    switch (this->port)
    {
        case Uart::UartPort::Uart1:
            drivers->uart.init<Uart::UartPort::Uart1, 115200>();
            break;
        case Uart::UartPort::Uart2:
            drivers->uart.init<Uart::UartPort::Uart2, 115200>();
            break;
        case Uart::UartPort::Uart3:
            drivers->uart.init<Uart::UartPort::Uart3, 115200>();
            break;
        case Uart::UartPort::Uart6:
            drivers->uart.init<Uart::UartPort::Uart6, 115200>();
            break;
        case Uart::UartPort::Uart7:
            drivers->uart.init<Uart::UartPort::Uart7, 115200>();
            break;
        case Uart::UartPort::Uart8:
            drivers->uart.init<Uart::UartPort::Uart8, 115200>();
            break;
        default:
            break;
    }
}

void DJISerial::updateSerial()
{
    switch (djiSerialRxState)
    {
        case SERIAL_HEADER_SEARCH:
        {
            // keep scanning for the head byte as long as you are here and have not yet found it.
            while (djiSerialRxState == SERIAL_HEADER_SEARCH && READ(&newMessage.header.headByte, 1))
            {
                // we found it, store the head byte
                if (newMessage.header.headByte == SERIAL_HEAD_BYTE)
                {
                    djiSerialRxState = PROCESS_FRAME_HEADER;
                    frameCurrReadByte = 0;
                }
            }
            break;
        }
        case PROCESS_FRAME_HEADER:  // the frame header consists of the length, type, and CRC8
        {
            frameCurrReadByte += READ(
                reinterpret_cast<uint8_t *>(&newMessage) + frameCurrReadByte + 1,
                sizeof(newMessage.header) - frameCurrReadByte - 1);

            // We have the complete message header in the frameHeader buffer
            if (frameCurrReadByte == sizeof(newMessage.header) - 1)
            {
                frameCurrReadByte = 0;

                // check crc8 on header
                if (rxCrcEnabled)
                {
                    // don't look at crc8 or frame type when calculating crc8
                    if (!verifyCRC8(
                            reinterpret_cast<uint8_t *>(&newMessage),
                            sizeof(newMessage.header) - 1,
                            newMessage.header.CRC8))
                    {
                        djiSerialRxState = SERIAL_HEADER_SEARCH;
                        RAISE_ERROR(drivers, "CRC8 failure");
                        return;
                    }
                }

                if (newMessage.header.dataLength >= SERIAL_RX_BUFF_SIZE)
                {
                    djiSerialRxState = SERIAL_HEADER_SEARCH;
                    RAISE_ERROR(drivers, "received message length longer than allowed max");
                    return;
                }

                // move on to processing message body
                djiSerialRxState = PROCESS_FRAME_DATA;
            }
            break;
        }
        case PROCESS_FRAME_DATA:  // READ bulk of message
        {
            int bytesToRead = sizeof(newMessage.messageType) + newMessage.header.dataLength;
            if (rxCrcEnabled)
            {
                bytesToRead += 2;
            }

            frameCurrReadByte += READ(
                reinterpret_cast<uint8_t *>(&newMessage) + sizeof(newMessage.header) +
                    frameCurrReadByte,
                bytesToRead - frameCurrReadByte);

            if (frameCurrReadByte == bytesToRead)
            {
                if (rxCrcEnabled)
                {
                    // move crc16 to `CRC16` position (currently in the data section if <
                    // SERIAL_RX_BUFF_SIZE length sent)
                    memcpy(
                        &newMessage.CRC16,
                        newMessage.data + newMessage.header.dataLength,
                        sizeof(newMessage.CRC16));

                    if (newMessage.CRC16 !=
                        algorithms::calculateCRC16(
                            reinterpret_cast<uint8_t *>(&newMessage),
                            sizeof(newMessage.header) + sizeof(newMessage.messageType) +
                                newMessage.header.dataLength))
                    {
                        djiSerialRxState = SERIAL_HEADER_SEARCH;
                        RAISE_ERROR(drivers, "CRC16 failure");
                        return;
                    }
                }

                mostRecentMessage = newMessage;

                messageReceiveCallback(mostRecentMessage);

                djiSerialRxState = SERIAL_HEADER_SEARCH;
            }
            else if (frameCurrReadByte > bytesToRead)
            {
                frameCurrReadByte = 0;
                RAISE_ERROR(drivers, "Invalid message length");
                djiSerialRxState = SERIAL_HEADER_SEARCH;
            }
            break;
        }
    }
}

}  // namespace tap::communication::serial