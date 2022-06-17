#include "chassis_tokyo_command.hpp"

#include <subsystems/chassis/chassis_rel_drive.hpp>

namespace src::Chassis {

ChassisTokyoCommand::ChassisTokyoCommand(src::Drivers* drivers, ChassisSubsystem* chassis, src::Gimbal::GimbalSubsystem* gimbal)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisTokyoCommand::initialize() {
}

void ChassisTokyoCommand::execute() {
    if (gimbal->isOnline()) {
        
    }
}

void ChassisTokyoCommand::end(bool) {
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisTokyoCommand::isReady() {
    return true;
}

bool ChassisTokyoCommand::isFinished() const {
    return false;
}

}  // namespace src::Chassis