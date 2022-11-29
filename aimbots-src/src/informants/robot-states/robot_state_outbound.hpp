#pragma once

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"
#include "src/communication/serial/ref_serial.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
#include "robot_state_interface.hpp"

namespace src::robotStates {

class RobotStateOutBound : public modm::pt::Protothread {
private:
    src::Drivers* drivers;
    tap::communication::serial::RefSerialTransmitter refSerial;
    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;

    RobotStateInterface* robotStateInterface;

    uint_16 message{};

// #ifdef TARGET_SENTRY
    static constexpr uint16_t SENTRY_REQUEST_ROBOT_ID = 0x200;

// #endif
public:
    RobotStateOutBound(src::Drivers* drivers);

    bool send();

    void updateQue();
}

}  // namespace src::robotStates
