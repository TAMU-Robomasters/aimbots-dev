#include "jetson_communicator.hpp"

#include <drivers.hpp>
#include <modm/platform/uart/uart_1.hpp>
#include <chrono>

#define READ(data, length) drivers->uart.read(JETSON_UART_PORT, data, length)
#define WRITE(data, length) drivers->uart.write(JETSON_UART_PORT, data, length)

namespace src::vision {

    JetsonCommunicator::JetsonCommunicator(src::Drivers * drivers)
        : drivers(drivers),
          //   rawSerialByte(0),
          //   messageBuffer(visionBuffer<512>(JETSON_END_BYTE)),
          lastMessage(),
          currentSerialState(JetsonCommunicatorSerialState::SearchingForMagic),
          nextByteIndex(0),
          jetsonOfflineTimeout() {}

    void JetsonCommunicator::initialize() {
        jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);
        drivers->uart.init<JETSON_UART_PORT, JETSON_BAUD_RATE>();
    }

    uint8_t displayBuffer[JETSON_MESSAGE_SIZE];

    float yawOffsetDisplay = 0;
    float pitchOffsetDisplay = 0;
    CVState cvStateDisplay = CVState::CV_STATE_PATROL;
    int readUnequal = 0;

    int lastMsgTime = 0;
    int msBetweenLastMessage = 0;

    /**
     * @brief Need to use modm's uart functions to read from the Jetson.
     * The Jetson sends information in the form of a JetsonMessage.
     * 
     * modm currently loads received bytes into an internal buffer, accessible using the READ() call.
     * When we receive the message-agnostic end byte we unload from the buffer, check the message length, and reinterpret a JetsonMessage from our received bytes.
     */
    void JetsonCommunicator::updateSerial() {
        switch (currentSerialState) {
            case JetsonCommunicatorSerialState::SearchingForMagic: {
                size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);
                if (bytesRead != 1) return;

                // We've gotten data from the Jetson, so we can restart this.
                jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);

                // little endian moment
                // The the first byte in the message will be the LSB of the magic number,
                // so we only have to and it with 0xff, rather than shifting it right first.
                if (rawSerialBuffer[nextByteIndex] == ((JETSON_MESSAGE_MAGIC >> (8 * nextByteIndex)) & 0xff))
                    nextByteIndex++;

                if (nextByteIndex == sizeof(decltype(JETSON_MESSAGE_MAGIC))) {
                    // We know that the magic is right, so we can just change the state. If the
                    // magic number wasn't an exact match, we wouldn't have gotten this far.
                    currentSerialState = JetsonCommunicatorSerialState::AssemblingMessage;
                }
                break;
            }
            case JetsonCommunicatorSerialState::AssemblingMessage: {
                size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);
                if (bytesRead != 1) {
                    return;
                } else {
                    nextByteIndex++;
                }

                // We've gotten data from the Jetson, so we can restart this.
                jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);

                if (nextByteIndex == sizeof(JetsonMessage)) {
                    lastMessage = *reinterpret_cast<JetsonMessage*>(&rawSerialBuffer);

                    memcpy(displayBuffer, rawSerialBuffer, sizeof(displayBuffer));

                    msBetweenLastMessage = tap::arch::clock::getTimeMilliseconds() - lastMsgTime;
                    lastMsgTime = tap::arch::clock::getTimeMilliseconds();

                    yawOffsetDisplay = lastMessage.targetYawOffset;
                    pitchOffsetDisplay = lastMessage.targetPitchOffset;
                    cvStateDisplay = lastMessage.cvState;

                    // uint8_t* buffer = reinterpret_cast<uint8_t*>(&lastMessage);
                    // WRITE(buffer, sizeof(decltype(JETSON_MESSAGE_MAGIC)));
                    currentSerialState = JetsonCommunicatorSerialState::SearchingForMagic;
                    nextByteIndex = 0;
                }
                break;
            }
        }
    }
}
