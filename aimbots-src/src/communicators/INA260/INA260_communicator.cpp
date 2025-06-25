
#ifndef ALL_SENTRIES

#include "INA260_communicator.hpp"

#include <drivers.hpp>

#define READ(data, length) drivers->uart.read(INA260_UART_PORT, data, length)
#define WRITE(data, length) drivers->uart.write(INA260_UART_PORT, data, length)

namespace src::Informants::INA260 {

float powerDisplay = 0.0f;
float voltage_mVDisplay = 0.0f; 
float current_mADisplay = 0.0f;

INA260Communicator::INA260Communicator(src::Drivers* drivers)
    : drivers(drivers),
      currentSerialState(INA260CommunicatorSerialState::SearchingForMagic),
      nextByteIndex(0),
      INA260OfflineTimeout(),
      lastMessage()
{}

void INA260Communicator::initialize() {
    INA260OfflineTimeout.restart(INA260_OFFLINE_TIMEOUT_MILLISECONDS);
    drivers->uart.init<INA260_UART_PORT, INA260_BAUD_RATE>();
}

uint8_t displayBuffer[INA260_MESSAGE_SIZE];
int displayBufIndex = 0;

int lastMsgTimeDisplay = 0;
int msBetweenLastMessageDisplay = 0;

/**
 * @brief Rewrote jetson communictor to use UART to 
 * recieve data from esp32 that sends INA260 data
 * Last mintue comp decision
 */
alignas(INA260Message) uint8_t rawSerialDisplay[sizeof(INA260Message)];

void INA260Communicator::updateSerial() {
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);  // attempts to pull one byte from the buffer
    if (bytesRead != 1) return;

    INA260OfflineTimeout.restart(INA260_OFFLINE_TIMEOUT_MILLISECONDS);

    displayBuffer[displayBufIndex] = rawSerialBuffer[0];            // copy byte to display buffer
    displayBufIndex = (displayBufIndex + 1) % INA260_MESSAGE_SIZE;  // increment display index and wrap around if necessary

    switch (currentSerialState) {
        // ...looking for message start...
        case INA260CommunicatorSerialState::SearchingForMagic: {
            // Check if the byte we just read is the byte we expected in the magic number.
            if (rawSerialBuffer[nextByteIndex] == ((INA260_MESSAGE_MAGIC >> (8 * nextByteIndex)) & 0xff)) {
                nextByteIndex++;
            } else {
                nextByteIndex = 0;  // if not, reset the index and start over.
            }

            // Wait until we've reached the end of the magic number. If any of the bytes in the magic number weren't a match,
            // we wouldn't have gotten this far.
            if (nextByteIndex == sizeof(decltype(INA260_MESSAGE_MAGIC))) {
                currentSerialState = INA260CommunicatorSerialState::AssemblingMessage;
            }
            break;
        }
        // ...found message start, assemble message...
        case INA260CommunicatorSerialState::AssemblingMessage: {
            nextByteIndex++;

            // Increment the byte index until we reach the expected end of a message, then parse the message.
            if (nextByteIndex == INA260_MESSAGE_SIZE) {
                // Reinterpret the received bytes into a JetsonMessage
                lastMessage = *reinterpret_cast<INA260Message*>(&rawSerialBuffer);
                powerDisplay = lastMessage.power;
                voltage_mVDisplay = lastMessage.voltage_mV;
                current_mADisplay = lastMessage.current_mA;

                // Update last message time and time between last message and now.
                if (lastMsgTimeDisplay == 0) {
                    lastMsgTimeDisplay = tap::arch::clock::getTimeMilliseconds();
                } else {
                    msBetweenLastMessageDisplay =
                        currTime - lastMsgTimeDisplay;  // Should be pretty close to the message send rate.
                    lastMsgTimeDisplay = currTime;
                }

                // As we've received a full message, reset the byte index and go back to searching for the magic number.
                nextByteIndex = 0;
                currentSerialState = INA260CommunicatorSerialState::SearchingForMagic;
            } else {
                rawSerialDisplay[nextByteIndex] = rawSerialBuffer[nextByteIndex];
            }

            break;
        }
    }

}

INA260Message const& INA260Communicator::getLastValidMessage() const {
    return lastMessage;
}

// float INA260Communicator::twoBytesToFloat(uint16_t bytes) const{
//     uint32_t empty = 0x00000000;
//     uint32_t firstSignBit = empty;
//     uint32_t mantissaBits = empty;
//     uint32_t exponentBits = empty;
//     uint32_t floatBits = empty;

//     firstSignBit |= bytes & 0x8000;
//     firstSignBit = firstSignBit << 16; // copy first bit

//     exponentBits |= bytes & 0x7C00;
//     exponentBits = exponentBits >> 10;
//     int32_t exponent = ((int32_t)exponentBits) - 15;  // subtract half-precision bias
//     exponent += 127; // add single percision bias
//     exponentBits = ((uint32_t)exponent) << 23; // move to position

//     mantissaBits |= bytes & 0x03FF;
//     mantissaBits = mantissaBits << 13; // start mantissa bits on 9th bit

//     floatBits = firstSignBit | exponentBits | mantissaBits;

//     return * (float *) &floatBits; // Carmack style 😎
// }

}  // namespace src::Informants::INA260
#endif // ALL_SENTRIES