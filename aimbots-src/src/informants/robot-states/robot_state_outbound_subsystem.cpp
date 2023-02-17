 #include "robot_state_outbound_subsystem.hpp"

#include "drivers.hpp"

namespace src::robotStates {
RobotStateOutBoundSubsystem::RobotStateOutBoundSubsystem(src::Drivers *drivers) : Subsystem(drivers), robotTransmiter(drivers) {}

void RobotStateOutBoundSubsystem::refresh() { robotTransmiter.send(); }
}  // namespace src::robotState