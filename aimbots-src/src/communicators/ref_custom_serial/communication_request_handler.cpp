#include "communication_request_handler.hpp"

#include "tap/errors/create_errors.hpp"

#include "informants/robot-states/robot_state.hpp"
// #include "informants/robot-states/robot_state_inbound.cpp"
#include "informants/robot-states/robot_state_interface.hpp"

#include "communication_message.hpp"
#include "drivers.hpp"

#ifdef REF_COMM_COMPATIBLE

using namespace src::RobotStates;

namespace src::Communication {

CommunicationRequestHandler::CommunicationRequestHandler(src::Drivers* drivers) : drivers(drivers) {}

void CommunicationRequestHandler::operator()(const tap::communication::serial::DJISerial::ReceivedSerialMessage& message) {
    MessageType type =
        static_cast<MessageType>(message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);

    switch (type) {
#ifdef ALL_SENTRIES
        // case MessageType::TEAM_MESSAGE_STANDARD:
        //     if (teamMesssageHandlerStandard != nullptr) {
        //         updateRobotStateStandard();
        //     }
        //     break;
        // case MessageType::TEAM_MESSAGE_HERO:
        //     if (teamMesssageHandlerHero != nullptr) {
        //         updateRobotStateHero();
        //     }
        //     break;
        // case MessageType::TEAM_MESSAGE_SENTRY:
        //     if (teamMessageHandlerSentry != nullptr) {
        //         updateRobotStateSentry();
        //     }
        //     break;

#else
        case MessageType::ROBOT_STATE:
            break;
#endif

        default:
            RAISE_ERROR(drivers, "invalid message tpye");
            break;
    }
}

}  // namespace src::Communication

#endif  // #ifdef REF_COMM_COMPATIBLE