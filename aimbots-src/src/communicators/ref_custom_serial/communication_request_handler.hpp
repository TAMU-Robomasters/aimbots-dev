#pragma once

#include "tap/communication/serial/ref_serial.hpp"
// #include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "informants/robot-states/robot_state.hpp"
#include "informants/robot-states/robot_state_interface.hpp"
#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "drivers.hpp"

#ifdef REF_COMM_COMPATIBLE

namespace src::Communication {
class CommunicationRequestHandler : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler {
public:
    using MessageReceivedCallback = void (*)();
    void operator()(const tap::communication::serial::DJISerial::ReceivedSerialMessage& message) override final;

    CommunicationRequestHandler(src::Drivers* drivers);

    bool recive();

    void updateStates();
#ifdef ALL_SENTRIES
    void attachTeamMessageHandlerStandard(MessageReceivedCallback message) { teamMesssageHandlerStandard = message; }
    void attachTeamMessageHandlerHero(MessageReceivedCallback message) { teamMesssageHandlerHero = message; }

#else
    void attachRobotStateHandler(MessageReceivedCallback message) { robotStateHandler = message; }
    void attachEnemyRobotStateHandler(MessageReceivedCallback message) { enemyRobotStateHandler = message; }
#endif
private:
    src::Drivers* drivers;

#ifdef ALL_SENTRIES
    MessageReceivedCallback teamMesssageHandlerStandard = nullptr;
    MessageReceivedCallback teamMesssageHandlerHero = nullptr;
    MessageReceivedCallback teamMessageHandlerSentry = nullptr;
#else
    MessageReceivedCallback robotStateHandler = nullptr;
    MessageReceivedCallback enemyRobotStateHandler = nullptr;
#endif
};
}  // namespace src::Communication

#endif  // #ifdef REF_COMM_COMPATIBLE