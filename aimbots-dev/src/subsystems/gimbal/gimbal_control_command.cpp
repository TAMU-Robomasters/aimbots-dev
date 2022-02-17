#include "gimbal_control_command.hpp"

namespace src::Gimbal {

GimbalControlCommand::GimbalControlCommand(src::Drivers* drivers, GimbalSubsystem* gimbalSubsystem)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem) { }

void GimbalControlCommand::initialize() { }

void GimbalControlCommand::execute() { }

void GimbalControlCommand::end(bool interrupted) { }

bool GimbalControlCommand::isFinished() const { }

}  // namespace src::Gimbal