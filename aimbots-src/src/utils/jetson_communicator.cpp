#include "jetson_communicator.hpp"

#include <drivers.hpp>
#include <modm/platform/uart/uart_1.hpp>

#define READ(data, length) drivers->uart.read(JETSON_UART_PORT, data, length)
#define WRITE(data, length) drivers->uart.write(JETSON_UART_PORT, data, length)

namespace utils {

JetsonCommunicator::JetsonCommunicator(src::Drivers* drivers)
    : drivers(drivers),
      lastMessage(),
      currentSerialState(JetsonCommunicatorSerialState::SearchingForMagic),
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
size_t rxBufferSize = 0;

/**
 * @brief Need to use modm's uart functions to read from the Jetson.
 * The Jetson sends information in the form of a JetsonMessage.
 */
void JetsonCommunicator::updateSerial() {
}
}