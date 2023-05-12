#pragma once
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/subsystem.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "communication_message.hpp"
#include "drivers.hpp"

namespace src::Communication {

class CommunicationResponseHandler : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler {
public:
    CommunicationResponseHandler(tap::Drivers &drivers);
    void operator()(const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) override final;

    
private:
    tap::Drivers &drivers;
};
}  // namespace src::Communication
