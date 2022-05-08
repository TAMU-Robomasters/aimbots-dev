#include "gimbal_cv_command.hpp"

#include <utils/jetson_protocol.hpp>

namespace src::Gimbal {

GimbalCVCommand::GimbalCVCommand(src::Drivers* drivers, GimbalSubsystem* gimbal, GimbalChassisRelativeController* controller)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbal),
      controller(controller) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalCVCommand::initialize() {}

void GimbalCVCommand::execute() {
    float yawOffset = drivers->cvCommunicator.lastValidMessage().targetYawOffset;
    float pitchOffset = drivers->cvCommunicator.lastValidMessage().targetPitchOffset;

    // TODO: We should patrol here, rather than abandoning the exec.
    if(drivers->cvCommunicator.lastValidMessage().cvState == ::utils::CV_STATE_PATROL) return;

    float targetYawAngle = gimbal->getCurrentYawAngle(AngleUnit::Radians)
                         - drivers->cvCommunicator.lastValidMessage().targetYawOffset * YAW_MOTOR_DIRECTION;
    controller->runYawController(AngleUnit::Radians, targetYawAngle);

    float targetPitchAngle = gimbal->getCurrentPitchAngle(AngleUnit::Radians)
                           - drivers->cvCommunicator.lastValidMessage().targetPitchOffset * getPitchMotorDirection();
    controller->runYawController(AngleUnit::Radians, targetYawAngle);
}

}