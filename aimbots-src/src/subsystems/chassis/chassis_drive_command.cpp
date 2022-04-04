#include "chassis_drive_command.hpp"

#include <subsystems/chassis/chassis_rel_drive.hpp>

namespace src::Chassis {

ChassisDriveCommand::ChassisDriveCommand(src::Drivers* drivers, ChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisDriveCommand::initialize() {}

void ChassisDriveCommand::execute() {
    Movement::Relative::onExecute(drivers, chassis);
}

void ChassisDriveCommand::end(bool) {}

bool ChassisDriveCommand::isReady() {
    return true;
}

bool ChassisDriveCommand::isFinished() const {
    return false;
}

}  // namespace src::Chassis