#include "communication_response_handler.hpp"

#include "tap/errors/create_errors.hpp"

#include "drivers.hpp"

namespace src::Communication {

CommunicationResponseHandler::CommunicationResponseHandler(src::Drivers &drivers) : drivers(drivers){};

void CommunicationResponseHandler::operator()(const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) {
    if (message.header.dataLength != sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader) + 1) {
        RAISE_ERROR((&drivers), "message length incorrect");
        return;
    }
    
    // this->sentryMoving = static_cast<bool>(message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);
}

}  // namespace src::Communication