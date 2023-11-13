#include "communication_request_handler.hpp"

#include "tap/errors/create_errors.hpp"

#include "informants/robot-states/robot_state.hpp"
// #include "informants/robot-states/robot_state_inbound.cpp"
#include "informants/robot-states/robot_state_interface.hpp"

#include "communication_message.hpp"
#include "drivers.hpp"

#ifdef REF_COMM_COMPATIBLE


namespace src::Communication {

CommunicationRequestHandler::CommunicationRequestHandler(src::Drivers* drivers) : drivers(drivers) {}

void CommunicationRequestHandler::operator()(const tap::communication::serial::DJISerial::ReceivedSerialMessage& message) {
    MessageType type = static_cast<MessageType>(message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);

    switch (type) {

        default:
            RAISE_ERROR(drivers, "invalid message tpye");
            break;
    }
}

}  // namespace src::Communication

#endif // #ifdef REF_COMM_COMPATIBLE