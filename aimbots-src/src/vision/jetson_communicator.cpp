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

    uint8_t displayBuffer[sizeof(JetsonMessage)];

    float yawOffsetDisplay = 0;
    float pitchOffsetDisplay = 0;
    CVState cvStateDisplay = CVState::CV_STATE_PATROL;
    int rxBufferSize = 0;
    uint8_t rawByteDisplay = 0;
    size_t bytesReadDisplay = 0;

    /**
     * @brief Need to use modm's uart functions to read from the Jetson.
     * The Jetson sends information in the form of a JetsonMessage.
     * 
     * modm currently loads received bytes into an internal buffer, accessible using the READ() call.
     * When we receive the message-agnostic end byte we unload from the buffer, check the message length, and reinterpret a JetsonMessage from our received bytes.
     */
    void JetsonCommunicator::updateSerial() {
        // size_t bytesRead = READ(&rawSerialByte[0], 1);
        // bytesReadDisplay = bytesRead;
        // if (bytesRead == 0) {
        //     return;
        // }

        while (READ(&rawSerialByte, 1) > 0) {
            bytesReadDisplay++;
            rawByteDisplay = rawSerialByte;
            if (messageBuffer.enqueue(rawSerialByte)) {
                std::pair<uint8_t*, size_t> rawMsg = messageBuffer.getLastMsg();
                rxBufferSize = rawMsg.second;
                if (rawMsg.second == 10) {
                    lastMessage = *reinterpret_cast<JetsonMessage*>(rawMsg.first);
                    yawOffsetDisplay = lastMessage.targetYawOffset;
                    pitchOffsetDisplay = lastMessage.targetPitchOffset;
                    cvStateDisplay = lastMessage.cvState;
                    // rxBufferSize = messageBuffer.size();
                }
                // switch (rawMsg.first[0]) {
                //     case aimAtTarget:
                //         rxBufferSize = rawMsg.second;
                //     default:
                //         break;
                // }
            }
        }
    }
}