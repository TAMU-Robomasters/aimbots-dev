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

#ifndef __SERIAL_TEST_CLASS_HPP__
#define __SERIAL_TEST_CLASS_HPP__

#include "dji_serial.hpp"

namespace aruwlib
{
class Drivers;
namespace serial
{
/**
 * A simple serial tester to insure `DjiSerial` is working properly.
 *
 * @note to test with a single MCB, instantiate a test class and connect
 *      TX to RX. You should be able to check if messages are being received.
 *      Additionally, watch `i` to check if you are dropping messages.
 */
class SerialTestClass : public DJISerial
{
public:
    /**
     * Attaches this test class to `Uart2`.
     */
    SerialTestClass(Drivers* drivers);

    DISALLOW_COPY_AND_ASSIGN(SerialTestClass)

    /**
     * Stores the sequenceNumber in `messageId`.
     */
    void messageReceiveCallback(const SerialMessage& completeMessage) override;

    /**
     * Sends a message of length 1, the byte `60`, with the `sequenceNumber` incremented.
     */
    void sendMessage();

private:
    uint8_t messageId;

    uint8_t i;
};

}  // namespace serial

}  // namespace aruwlib

#endif
