#include "gimbal_cv_command.hpp"

#include <vision/jetson_protocol.hpp>

namespace src::Gimbal {

    GimbalCVCommand::GimbalCVCommand(src::Drivers * drivers, GimbalSubsystem * gimbal, GimbalChassisRelativeController * controller)
        : tap::control::Command(),
          drivers(drivers),
          gimbal(gimbal),
          controller(controller) {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
    }

    void GimbalCVCommand::initialize() {}

    void GimbalCVCommand::execute() {
        float yawOffset = drivers->cvCommunicator.lastValidMessage().targetYawOffset * YAW_MOTOR_DIRECTION;
        float pitchOffset = drivers->cvCommunicator.lastValidMessage().targetPitchOffset * getPitchMotorDirection();

        // TODO: We should patrol here, rather than abandoning the exec.
        if (drivers->cvCommunicator.lastValidMessage().cvState == vision::CV_STATE_PATROL) return;

        float targetYawAngle = gimbal->getCurrentYawAngle(AngleUnit::Radians) - yawOffset;
        controller->runYawController(AngleUnit::Radians, targetYawAngle);

        float targetPitchAngle = gimbal->getCurrentPitchAngle(AngleUnit::Radians) - pitchOffset;
        controller->runYawController(AngleUnit::Radians, targetPitchAngle);
    }
}