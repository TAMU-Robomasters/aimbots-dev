#include "communication_response_handler.hpp"

#include "tap/errors/create_errors.hpp"

// #include "communication_message.hpp"
#include "informants/robot-states/robot_state.hpp"

#include "communication_message.hpp"
#include "communication_response_subsytem.hpp"
#include "drivers.hpp"

namespace src::Communication {

CommunicationResponseHandler::CommunicationResponseHandler(tap::Drivers &drivers) : drivers(drivers){};
uint8_t messageTest;
uint16_t messageType;
uint16_t CRC16;
uint16_t messageLengthDisplay;
uint16_t sizeInteractiveHeader;
bool error;

void CommunicationResponseHandler::operator()(const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) {
    error = false;
    messageLengthDisplay = message.header.dataLength;
    sizeInteractiveHeader = sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader) + 1;
    if (message.header.dataLength != sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader) + 1) {
        RAISE_ERROR((&drivers), "message length incorrect");
        error = true;
        return;
    }

    // message.CRC16[0];

    messageTest = static_cast<int>(message.data[1]);
    messageType = message.messageType;
    CRC16 = message.CRC16;

    // this->sentryMoving = static_cast<bool>(message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);
}

}  // namespace src::Communication