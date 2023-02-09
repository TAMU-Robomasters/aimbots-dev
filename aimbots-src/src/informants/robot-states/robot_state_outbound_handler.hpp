#pragma once

#include "tap/communication/serial/ref_serial.hpp"
// #include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
#include "robot_state_interface.hpp"
namespace src::robotStates {
class RobotStateOutBoundHandler : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler {
public:
    using MessageReceivedCallback = void (*)();
    void operator()(const tap::communication::serial::DJISerial::ReceivedSerialMessage& message) override final;

    RobotStateOutBoundHandler(src::Drivers* drivers);

    bool recive();

    void updateStates();
#ifdef TARGET_SENTRY
    void attachTeamMessageHandlerStandard(MessageReceivedCallback message) { teamMesssageHandlerStandard = message; }
    void attachTeamMessageHandlerHero(MessageReceivedCallback message) { teamMesssageHandlerHero = message; }

#else
    void attachRobotStateHandler(MessageReceivedCallback message) { robotStateHandler = message; }
    void attachEnemyRobotStateHandler(MessageReceivedCallback message) { enemyRobotStateHandler = message; }
#endif
private:
    src::Drivers* drivers;

#ifdef TARGET_SENTRY
    MessageReceivedCallback teamMesssageHandlerStandard = nullptr;
    MessageReceivedCallback teamMesssageHandlerHero = nullptr;
#else
    MessageReceivedCallback robotStateHandler = nullptr;
    MessageReceivedCallback enemyRobotStateHandler = nullptr;
#endif
};
}  // namespace src::robotStates