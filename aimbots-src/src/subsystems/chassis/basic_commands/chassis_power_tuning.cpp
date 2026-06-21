
#include "chassis_power_tuning.hpp"

#ifdef GIMBAL_UNTETHERED
#ifdef CHASSIS_COMPATIBLE

#include "subsystems/chassis/control/chassis_helper.hpp"

namespace src::Chassis {



ChassisTuningCommand::ChassisTuningCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis),
      rotationController(ROTATION_POSITION_PID_CONFIG)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisTuningCommand::initialize() {}



void ChassisTuningCommand::execute() {
        
}

void ChassisTuningCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisTuningCommand::isReady() { return true; }

bool ChassisTuningCommand::isFinished() const { return false; }

}  // namespace src::Chassis
#endif  //#ifdef CHASSIS_COMPATIBLE

#endif  //#ifdef GIMBAL_UNTETHERED