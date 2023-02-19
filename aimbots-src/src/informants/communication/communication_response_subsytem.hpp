#pragma once

#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/subsystem.hpp"

#include "modm/processing/protothread.hpp"

#include "drivers.hpp"

namespace src::Communication {
class CommunicationResponseSubsytem : public tap::control::Subsystem, ::modm::pt::Protothread {
public:
    CommunicationResponseSubsytem(src::Drivers &drivers);

    void refresh() override;

private:
    src::Drivers &drivers;

    tap::communication::serial::RefSerialTransmitter refSerialTransmitter;
    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;

    bool run();
};
}  // namespace src::Communication
