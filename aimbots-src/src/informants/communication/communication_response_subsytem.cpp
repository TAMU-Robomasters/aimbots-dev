#include "communication_response_subsytem.hpp"

#include "informants/robot-states/robot_state.hpp"

#include "communication_message.hpp"
#include "communication_request_handler.hpp"
#include "drivers.hpp"

namespace src::Communication {

CommunicationResponseSubsytem::CommunicationResponseSubsytem(tap::Drivers &drivers)
    : tap::control::Subsystem(&drivers),
      drivers(drivers),
      refSerialTransmitter(&drivers)  //,
//   states()
{
    // states = new RobotStates();
}

void CommunicationResponseSubsytem::refresh() { this->run(); }

uint8_t test = 0;
uint16_t datalength;
uint8_t subsytemCRC8;
uint16_t healthCheck;
bool CommunicationResponseSubsytem::run() {
#ifdef TARGET_SENTRY
    // ms = states.createMessage();
    // uint16_t sx = ms.standardX;
    test = static_cast<unsigned char>(rand() % 0xff);
    this->robotToRobotMessage.dataAndCRC16[1] = test;  // static_cast<uint8_t>(sx);
    // sx = sx >> 8;
    this->robotToRobotMessage.dataAndCRC16[2] = static_cast<unsigned char>(rand() % 0xff);  // static_cast<uint8_t>(sx);
    // uint16_t sy = ms.standardY;
    this->robotToRobotMessage.dataAndCRC16[3] = static_cast<unsigned char>(rand() % 0xff);  // static_cast<uint8_t>(sy);
    // sy = sy >> 8;
    this->robotToRobotMessage.dataAndCRC16[4] = static_cast<unsigned char>(rand() % 0xff);  // static_cast<uint8_t>(sy);

#endif

    datalength = robotToRobotMessage.frameHeader.dataLength;
    subsytemCRC8 = robotToRobotMessage.frameHeader.CRC8;

    healthCheck = drivers.refSerial.getRobotData().allRobotHp.red.sentry7;
    // short standardY = 0;
    // this->robotToRobotMessage.dataAndCRC16[3] = static_cast<uint8_t>(standardY);
    // standardY = standardY >> 8;
    // this->robotToRobotMessage.dataAndCRC16[4] = static_cast<uint8_t>(standardY);

    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());

    while (true) {
        this->robotToRobotMessage.dataAndCRC16[0] = static_cast<uint8_t>(rand() % 0xff);        // static_cast<uint8_t>(false);
        this->robotToRobotMessage.dataAndCRC16[1] = static_cast<uint8_t>(rand() % 0xff);        // static_cast<uint8_t>(sx);
        this->robotToRobotMessage.dataAndCRC16[2] = static_cast<unsigned char>(rand() % 0xff);  // static_cast<uint8_t>(sx);
        this->robotToRobotMessage.dataAndCRC16[3] = static_cast<unsigned char>(rand() % 0xff);  // static_cast<uint8_t>(sy);
        this->robotToRobotMessage.dataAndCRC16[4] = static_cast<unsigned char>(rand() % 0xff);  // static_cast<uint8_t>(sy);

        // if (this->sentryMoving != this->getDriveStatus()) {
        //     this->sentryMoving = this->getDriveStatus();
        // short standardX = 0;

#ifdef TARGET_STANDARD

        // PT_CALL(refSerialTransmitter.sendRobotToRobotMsg(
        //     &this->robotToRobotMessage,
        //     SENTRY_RESPONSE_MESSAGE_ID,
        //     drivers.refSerial.getRobotIdBasedOnCurrentRobotTeam(tap::communication::serial::RefSerialData::RobotId::BLUE_SENTINEL),
        //     4));

        PT_CALL(this->refSerialTransmitter.sendRobotToRobotMsg(
            &this->robotToRobotMessage,
            SENTRY_RESPONSE_MESSAGE_ID,
            drivers.refSerial.getRobotIdBasedOnCurrentRobotTeam(tap::communication::serial::RefSerialData::RobotId::BLUE_HERO),
            4));
        // }
        // #elif TARGET_HERO
        // this->robotToRobotMessage.dataAndCRC16[0] = static_cast<uint8_t>(false);

        // PT_CALL(this->refSerialTransmitter.sendRobotToRobotMsg(
        //     &this->robotToRobotMessage,
        //     SENTRY_REQUEST_ROBOT_ID,
        //     drivers.refSerial.getRobotIdBasedOnCurrentRobotTeam(tap::communication::serial::RefSerialData::RobotId::BLUE_SENTINEL),
        //     1));

#endif
        PT_YIELD();
    }

    PT_END();
}

}  // namespace src::Communication
