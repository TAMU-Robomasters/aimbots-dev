#pragma once

#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/subsystem.hpp"

#include "informants/robot-states/robot_state_interface.hpp"
#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "communication_message.hpp"
#include "drivers.hpp"

namespace src::Communication {
class CommunicationResponseSubsytem : public tap::control::Subsystem, ::modm::pt::Protothread {
public:
    CommunicationResponseSubsytem(tap::Drivers &drivers);

    void refresh() override;

private:
    tap::Drivers &drivers;

    tap::communication::serial::RefSerialTransmitter refSerialTransmitter;
    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;

    // RobotStates::RobotStates &states;

    // robot_state_message_team ms;

    bool run();
};
}  // namespace src::Communication
