#include "jetson_communicator.hpp"

#include <drivers.hpp>

#define READ(data, length) drivers->uart.read(JETSON_UART_PORT, data, length)

namespace utils {

JetsonCommunicator::JetsonCommunicator(src::Drivers* drivers)
    : drivers(drivers),
      lastMessage(),
      jetsonOfflineTimeout() { }

void JetsonCommunicator::initialize() {
    jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);
    drivers->uart.init<JETSON_UART_PORT, JETSON_BAUD_RATE>();
}

void JetsonCommunicator::updateSerial() {
    size_t bytesRead = READ(&rawSerialBuffer[0], 1);
    if (bytesRead != 1) return;

    // We've gotten our first data from the Jetson, so we can restart this.
    jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);

    // little endian moment
    // The the first byte in the message will be the LSB of the magic number,
    // so we only have to and it with 0xff, rather than shifting it right first.
    while(rawSerialBuffer[0] != (JETSON_MESSAGE_MAGIC & 0xff)) {
        bytesRead = READ(&rawSerialBuffer[0], 1);
        if(bytesRead != 1) return;
    }

    bytesRead = READ(&rawSerialBuffer[1], 7);
    if(bytesRead != 7) return;

    uint64_t readMagic = *reinterpret_cast<uint64_t*>(&rawSerialBuffer[0]);
    if(readMagic != JETSON_MESSAGE_MAGIC) return;

    bytesRead = READ(&rawSerialBuffer[8], sizeof(JetsonMessage) - 8);
    if(bytesRead != sizeof(JetsonMessage) - 8) return;

    lastMessage = *reinterpret_cast<JetsonMessage*>(&rawSerialBuffer);
}

}