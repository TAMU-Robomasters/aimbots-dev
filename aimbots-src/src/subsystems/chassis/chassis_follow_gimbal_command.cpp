#include "chassis_follow_gimbal_command.hpp"

#include <subsystems/chassis/chassis_rel_drive.hpp>

namespace src::Chassis {

ChassisFollowGimbalCommand::ChassisFollowGimbalCommand(src::Drivers* drivers, ChassisSubsystem* chassis, src::Gimbal::GimbalSubsystem* gimbal)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisFollowGimbalCommand::initialize() {
}

void ChassisFollowGimbalCommand::execute() {
    Movement::Direct::onExecute(drivers, chassis);
}

void ChassisFollowGimbalCommand::end(bool) {
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisFollowGimbalCommand::isReady() {
    return true;
}

bool ChassisFollowGimbalCommand::isFinished() const {
    return false;
}

}  // namespace src::Chassis