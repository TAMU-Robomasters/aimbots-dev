#pragma once

#include "tap/communication/serial/ref_serial.hpp"
// #include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"
#include "informants/robot-states/robot_state.hpp"
#include "informants/robot-states/robot_state_interface.hpp"

#include "drivers.hpp"

#ifdef REF_COMM_COMPATIBLE

namespace src::Communication {
class CommunicationRequestHandler : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler {
public:
    using MessageReceivedCallback = void (*)();
    void operator()(const tap::communication::serial::DJISerial::ReceivedSerialMessage& message) override final;

    CommunicationRequestHandler(src::Drivers* drivers);

    bool recive();

    void updateStates();

private:
    src::Drivers* drivers;

    MessageReceivedCallback robotStateHandler = nullptr;
    MessageReceivedCallback enemyRobotStateHandler = nullptr;
    
};
}  // namespace src::robotStates

#endif // #ifdef REF_COMM_COMPATIBLE