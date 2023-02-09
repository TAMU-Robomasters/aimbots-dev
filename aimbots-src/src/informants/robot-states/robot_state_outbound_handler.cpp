#include "robot_state_outbound_handler.hpp"

#include "tap/errors/create_errors.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
#include "robot_state_message.hpp"

namespace src::robotStates {

RobotStateOutBoundHandler::RobotStateOutBoundHandler(src::Drivers* drivers) : drivers(drivers) {}

void RobotStateOutBoundHandler::operator()(const tap::communication::serial::DJISerial::ReceivedSerialMessage& message) {
    MessageType type = static_cast<MessageType>(message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);

    switch (type) {
#ifdef TARGET_SENTRY
        case MessageType::TEAM_MESSAGE_STANDARD:
            break;

#else
        case MessageType::ROBOT_STATE:
            break;
#endif

        default:
            RAISE_ERROR(drivers, "invalid message tpye");
            break;
    }
}

}  // namespace src::robotStates