#include "chassis_manual_drive_command.hpp"

#include <subsystems/chassis/chassis_rel_drive.hpp>

namespace src::Chassis {

ChassisManualDriveCommand::ChassisManualDriveCommand(src::Drivers* drivers, ChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisManualDriveCommand::initialize() {
}

void ChassisManualDriveCommand::execute() {
    Movement::Relative::onExecute(drivers, chassis);
}

void ChassisManualDriveCommand::end(bool) {
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisManualDriveCommand::isReady() {
    return true;
}

bool ChassisManualDriveCommand::isFinished() const {
    return false;
}

}  // namespace src::Chassis