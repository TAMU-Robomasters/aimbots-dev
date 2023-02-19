#pragma once



#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "informants/robot-states/robot_state.hpp"
#include "informants/robot-states/robot_state_interface.hpp"
#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "drivers.hpp"

namespace src::Communication {

class CommunicationResponseHandler : tap::communication::serial::RefSerial::RobotToRobotMessageHandler {
public:
    CommunicationResponseHandler(src::Drivers &drivers);
    void operator()(const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) override final;

    private:
        src::Drivers &drivers;
};
}  // namespace src::Communication
