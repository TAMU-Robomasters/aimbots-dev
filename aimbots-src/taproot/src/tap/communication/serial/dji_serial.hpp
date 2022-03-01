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

#ifndef __serial_h_
#define __serial_h_

#include <cstdint>

#include "tap/algorithms/crc.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/util_macros.hpp"

namespace tap
{
class Drivers;
namespace serial
{
/**
 * A serial handler that implements a specific protocol to be used for
 * communicating with the referee system. Also used for our personal
 * communication with the xavier.
 *
 * Extend this class and implement messageReceiveCallback if you
 * want to use this serial protocol on a serial line.
 *
 * Structure of a Serial Message:
 * \rst
 * +-----------------+------------------------------------------------------------+
 * | Byte Number     | Byte Description                                           |
 * +=================+============================================================+
 * | Frame Header                                                                 |
 * +-----------------+------------------------------------------------------------+
 * | 0               | Frame Head Byte (0xA5)                                     |
 * +-----------------+------------------------------------------------------------+
 * | 1               | Frame Data Length, LSB                                     |
 * +-----------------+------------------------------------------------------------+
 * | 2               | Frame Data Length, MSB                                     |
 * +-----------------+------------------------------------------------------------+
 * | 3               | Frame Sequence Number                                      |
 * +-----------------+------------------------------------------------------------+
 * | 4               | CRC8 of the frame, (bytes 0 - 3)                           |
 * +-----------------+------------------------------------------------------------+
 * | 5               | Message Type, LSB                                          |
 * +-----------------+------------------------------------------------------------+
 * | 6               | Message Type, MSB                                          |
 * +-----------------+------------------------------------------------------------+
 * | Body - Data Length bytes                                                     |
 * +-----------------+------------------------------------------------------------+
 * | Message CRC                                                                  |
 * +-----------------+------------------------------------------------------------+
 * | 7 + Data Length | CRC16 of header and frame, LSB (bytes 0 - 6 + Data Length) |
 * +-----------------+------------------------------------------------------------+
 * | 8 + Data Length | CRC16 of header and frame, MSB                             |
 * +-----------------+------------------------------------------------------------+
 * \endrst
 */
class DJISerial
{
private:
    static const uint16_t SERIAL_RX_BUFF_SIZE = 256;
    static const uint16_t SERIAL_TX_BUFF_SIZE = 256;
    static const uint16_t SERIAL_HEAD_BYTE = 0xA5;
    static const uint8_t FRAME_DATA_LENGTH_OFFSET = 1;
    static const uint8_t FRAME_SEQUENCENUM_OFFSET = 3;
    static const uint8_t FRAME_CRC8_OFFSET = 4;
    static const uint8_t FRAME_HEADER_LENGTH = 7;
    static const uint8_t FRAME_TYPE_OFFSET = 5;
    static const uint8_t FRAME_CRC16_LENGTH = 2;
    static constexpr uint16_t SERIAL_RX_FRAME_HEADER_AND_BUF_SIZE =
        SERIAL_RX_BUFF_SIZE + FRAME_HEADER_LENGTH;

public:
    /**
     * A container for storing TX and RX messages.
     */
    struct SerialMessage
    {
        uint8_t headByte;  /// Use `SERIAL_HEAD_BYTE`.
        uint16_t length;   /// Must be less than SERIAL_RX_BUFF_SIZE or SERIAL_TX_BUFF_SIZE.
        uint16_t type;     /// The type is specified and interpreted by a derived class.
        uint8_t data[SERIAL_RX_BUFF_SIZE];
        uint32_t messageTimestamp;  /// The timestamp is in milliseconds.
        uint8_t sequenceNumber;     /// A derived class may increment this for debugging purposes.
    };

    /**
     * Construct a Serial object.
     *
     * @param[in] port serial port to work on.
     * @param[in] isRxCRCEnforcementEnabled `true` to enable Rx CRC Enforcement.
     */
    DJISerial(Drivers *drivers, Uart::UartPort port, bool isRxCRCEnforcementEnabled = true);
    DISALLOW_COPY_AND_ASSIGN(DJISerial)
    mockable ~DJISerial() = default;

    /**
     * Initialize serial. In particular, initializes the hardware serial
     * specified upon construction.
     *
     * @note currently, only uart ports 1, 2, and 6 are enabled. Be sure
     *      to add a serial port to `uart.hpp` if you want to use the serial.
     *      Also, if you add a new uart port to be generated in the `project.xml`
     *      file, you should add it to both the `Uart` class and this function.
     * @see `Uart`
     */
    mockable void initialize();

    /**
     * Receive messages. Call periodically in order to receive all
     * incoming messages.
     *
     * @note tested with a delay of 10 microseconds with referee system. The
     *      longer the timeout the more likely a message failure may occur.
     */
    mockable void updateSerial();

    /**
     * Called when a complete message is received. A derived class must
     * implement this in order to handle incoming messages properly.
     *
     * @param[in] completeMessage a reference to the full message that has
     *      just been received by this class.
     */
    virtual void messageReceiveCallback(const SerialMessage &completeMessage) = 0;

private:
    // RX related information.

    enum SerialRxState
    {
        SERIAL_HEADER_SEARCH,  /// A header byte has not yet been received.
        PROCESS_FRAME_HEADER,  /// A header is received and the frame header is being processed.
        PROCESS_FRAME_DATA     /// The data is being processed.
    };

    /// The serial port you are connected to.
    Uart::UartPort port;

    /// stuff for RX, buffers to store parts of the header, state machine.
    SerialRxState djiSerialRxState;

    /// Message in middle of being constructed.
    SerialMessage newMessage;

    /// Most recent complete message.
    SerialMessage mostRecentMessage;
    /**
     * The current zero indexed byte that is being read from the `Uart` class.
     * Reset after the header is read (so increments from 0 to the header length
     * then reset to 0 and increments again from 0 to the message length, or
     * message length + 2 if crc enforcement is enabled).
     */
    uint16_t frameCurrReadByte;
    /**
     * Stores the incoming header. After the frame is received, it is transferred to
     * `newMessage`.
     */
    uint8_t frameHeader[FRAME_HEADER_LENGTH];

    bool rxCrcEnabled;

    /**
     * Currently calculated crc16 value. The crc16 is computed it two parts - the header and
     * the main body. Between receiving the header and the body, we store the intermediate
     * crc value here.
     */
    uint16_t currCrc16 = 0;

    // TX related information.

    /// TX buffer.
    uint8_t txBuffer[SERIAL_TX_BUFF_SIZE];

    /**
     * Calculate CRC8 of given array and compare against expectedCRC8.
     *
     * @param[in] data array to calculate CRC8.
     * @param[in] length length of array to check.
     * @param[in] expectedCRC8 expected CRC8.
     * @return if the calculated CRC8 matches CRC8 given.
     */
    inline bool verifyCRC8(uint8_t *message, uint32_t messageLength, uint8_t expectedCRC8)
    {
        return tap::algorithms::calculateCRC8(message, messageLength) == expectedCRC8;
    }

protected:
    Drivers *drivers;

    /**
     * Subclasses can access the message that this class sends as to allow
     * for modification.
     */
    SerialMessage txMessage;

    /**
     * Send a Message. This constructs a message from the `txMessage`,
     * which is protected and should be manipulated by a derived class.
     *
     * @return true if succeed, false if failed.
     */
    bool send();
};

}  // namespace serial

}  // namespace tap

#endif
