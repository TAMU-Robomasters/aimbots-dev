#pragma once

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "drivers.hpp"
#include "ref_serial.hpp"
#include "robot_state.hpp"
#include "robot_state_interface.hpp"

namespace src::robotStates {

class RobotStateInBound : public modm::pt::Protothread {
private:
    src::Drivers* drivers;
    tap::communication::serial::RefSerialTransmitter refSerial;

    RobotStateInterface* robotStateInterface;

public:
    RobotStateInBound(src::Drivers* drivers);

    bool recive();

    void updateStates();

}

}  // namespace src::robotStates
