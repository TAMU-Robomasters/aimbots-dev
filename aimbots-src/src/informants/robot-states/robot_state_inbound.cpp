#include "robot_state_inbound.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
#include "robot_state_interface.hpp"

namespace src::robotStates {
RobotStateInBound(src::Drivers* drivers) : drivers(drivers), refSerial(drivers) {}

bool recive() { return false; }

void updateStates() {}
}  // namespace src::robotStates