#include "jetson_communicator.hpp"

#include <drivers.hpp>
#include <modm/platform/uart/uart_1.hpp>
#include <chrono>

#define READ(data, length) drivers->uart.read(JETSON_UART_PORT, data, length)
#define WRITE(data, length) drivers->uart.write(JETSON_UART_PORT, data, length)

namespace src::vision {

    JetsonCommunicator::JetsonCommunicator(src::Drivers * drivers)
        : drivers(drivers),
          rawSerialByte(0),
          messageBuffer(visionBuffer<512>(JETSON_END_BYTE)),
          lastMessage(),
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
        if (modm::platform::Usart1::receiveBufferSize() >= JETSON_MESSAGE_SIZE) {
            msBetweenLastMessage = tap::arch::clock::getTimeMilliseconds() - lastMsgTime;
            lastMsgTime = tap::arch::clock::getTimeMilliseconds();

            uint8_t rawData[JETSON_MESSAGE_SIZE];
            size_t bytesRead = READ(rawData, JETSON_MESSAGE_SIZE);
            readUnequal += (bytesRead != JETSON_MESSAGE_SIZE);
        }
    }
}
