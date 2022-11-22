#include "robot_state_outbound.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
#include "robot_state_interface.hpp"


namespace src::robotStates {
RobotStateOutBound(src::Drivers* drivers) : drivers(drivers), refSerial(drivers) {}

bool send() { return false; }

void updateQue() {}
}  // namespace src::robotStates