#include "robot_state_inbound.hpp"

#include "tap/communication/serial/ref_serial.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
#include "robot_state_interface.hpp"

namespace src::robotStates {

// RobotStateInBound::RobotStateInBound(src::Drivers* drivers) : drivers(drivers), refSerial(drivers) {}

// bool recive() {
//     // return refSerial
//     return false;
// }

// void updateStates() {
//     if(recive()){
//         //TODO: check message type, depending on it do funny things

//     }

}  // namespace src::robotStates