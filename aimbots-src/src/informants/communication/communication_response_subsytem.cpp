#include "communication_response_subsytem.hpp"

#include "communication_message.hpp"
#include "drivers.hpp"

namespace src::Communication {

CommunicationResponseSubsytem::CommunicationResponseSubsytem(src::Drivers &drivers)
    : tap::control::Subsystem(&drivers),
      drivers(drivers),
      refSerialTransmitter(&drivers) {}

void CommunicationResponseSubsytem::refresh() { this->run(); }

bool CommunicationResponseSubsytem::run() {
    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());

    while (true) {
        // if (this->sentryMoving != this->getDriveStatus()) {
        //     this->sentryMoving = this->getDriveStatus();

        this->robotToRobotMessage.dataAndCRC16[0] = static_cast<uint8_t>(false);

        PT_CALL(refSerialTransmitter.sendRobotToRobotMsg(
            &this->robotToRobotMessage,
            SENTRY_RESPONSE_MESSAGE_ID,
            drivers.refSerial.getRobotIdBasedOnCurrentRobotTeam(tap::communication::serial::RefSerialData::RobotId::BLUE_HERO),
            1));

        PT_CALL(this->refSerialTransmitter.sendRobotToRobotMsg(
            &this->robotToRobotMessage,
            SENTRY_RESPONSE_MESSAGE_ID,
            drivers.refSerial.getRobotIdBasedOnCurrentRobotTeam(tap::communication::serial::RefSerialData::RobotId::BLUE_SOLDIER_1),
            1));
        // }

        PT_YIELD();
    }

    PT_END();
}

}  // namespace src::Communication
