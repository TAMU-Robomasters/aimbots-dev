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

#include "serial_test_class.hpp"

namespace tap
{
namespace serial
{
SerialTestClass::SerialTestClass(Drivers* drivers)
    : DJISerial(drivers, Uart::UartPort::Uart2),
      messageId(0),
      i(0)
{
}

void SerialTestClass::messageReceiveCallback(const SerialMessage& completeMessage)
{
    messageId = completeMessage.sequenceNumber;
}

void SerialTestClass::sendMessage()
{
    this->txMessage.length = 1;
    this->txMessage.headByte = 0xa5;
    this->txMessage.sequenceNumber = i;
    this->txMessage.type = 4;
    this->txMessage.data[0] = 60;
    this->send();
    i++;
}

}  // namespace serial

}  // namespace tap
