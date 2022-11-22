#pragma once

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "drivers.hpp"
#include "ref_serial.hpp"
#include "robot_state.hpp"
#include "robot_state_interface.hpp"

namespace src::robotStates {

class RobotStateOutBound : public modm::pt::Protothread {
private:
    src::Drivers* drivers;
    tap::communication::serial::RefSerialTransmitter refSerial;

    RobotStateInterface* robotStateInterface;

public:
    RobotStateOutBound(src::Drivers* drivers);

    bool send();

    void updateQue();

}

}  // namespace src::robotStates
