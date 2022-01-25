#include "chassis_drive_command.hpp"

namespace src::Chassis {

ChassisDriveCommand::ChassisDriveCommand(src::Drivers* drivers, ChassisSubsystem* chassisSubsystem)
    : TapCommand(),
      drivers(drivers),
      chassis(chassisSubsystem) {
    addSubsystemRequirement((tap::control::Subsystem*)chassis);
}

void ChassisDriveCommand::initialize() {}

void ChassisDriveCommand::execute() {}

void ChassisDriveCommand::end(bool interrupted) {}

bool ChassisDriveCommand::isReady() {
    return true;
}

bool ChassisDriveCommand::isFinished() const {
    return false;
}
}  // namespace src::Chassis