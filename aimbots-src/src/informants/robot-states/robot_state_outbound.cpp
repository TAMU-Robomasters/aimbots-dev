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

bool send() {
    // updateQue();

    // PT_CALL(refSerial.sendRobotToRobotMsg(
    //     &robotToRobotMessage,
    //     SENTRY_REQUEST_ROBOT_ID,
    //     drivers->refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerialData::RobotId::BLUE_SENTINEL),
    //     1));

    // PT_END();
    return false;
}

void updateQue() {
    // #ifdef TARGET_SENTRY
    //     Team color = drivers->getRobotData->robotData.robotId == RED_SENTINEL ? Team::RED : Team::BLUE;
    //     Team colorEnemy = color == TEAM::RED ? TEAM::BLUE : TEAM::RED;

    // #endif
}

}  // namespace src::robotStates