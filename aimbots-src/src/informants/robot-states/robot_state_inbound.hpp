#pragma once

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
#include "robot_state_interface.hpp"

namespace src::robotStates {

class RobotStateInBound : public modm::pt::Protothread {
private:
    src::Drivers* drivers;
    tap::communication::serial::RefSerialTransmitter refSerial;

    RobotStates* robotStateInterface;

public:
    RobotStateInBound(src::Drivers* drivers);

    bool recive();

    void updateStates();
};

}  // namespace src::robotStates
