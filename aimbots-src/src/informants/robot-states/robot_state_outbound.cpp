#include "robot_state_outbound.hpp"

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
#include "robot_state_interface.hpp"
#include "robot_state_message.hpp"

namespace src::robotStates {
RobotStateOutBound::RobotStateOutBound(src::Drivers* drivers) : drivers(drivers), refSerial(drivers) {}

bool RobotStateOutBound::send() {
    PT_BEGIN();
    // uint16_t lastMessage;
    while (true) {
        getNextMessageToSend();

        if (/*message != lastMessage*/ true) {
            PT_CALL(refSerial.sendRobotToRobotMsg(
                &robotToRobotMessage,
                SENTRY_REQUEST_ROBOT_ID,
                drivers->refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerialData::RobotId::BLUE_SENTINEL),
                1));
        } else {
            PT_YIELD();
        }
        updateQue();
        // lastMessage = message;
    }
    PT_END();
    return false;
}

void RobotStateOutBound::updateQue() {
#ifdef TARGET_SENTRY
    // Team color = drivers->refSerial->getRobotData().robotID == 7 ? Team::RED : Team::BLUE;
    Team color = Team::RED;
    // Team color = drivers->getRobotData->robotData.robotId == RED_SENTINEL ? Team::RED : Team::BLUE;
    Team colorEnemy = color == Team::RED ? Team::BLUE : Team::RED;

#endif
}

}  // namespace src::robotStates