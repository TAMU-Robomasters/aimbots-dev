#include "gimbal_chassis_relative_controller.hpp"

#include <utils/robot_specific_inc.hpp>
#include <subsystems/gimbal/controllers/gimbal_gravity_helper.hpp>

namespace src::Gimbal {

GimbalChassisRelativeController::GimbalChassisRelativeController(GimbalSubsystem* gimbalSubsystem)
    : gimbal(gimbalSubsystem),
      yawPID(
        POSITION_PID_KP,
        POSITION_PID_KI,
        POSITION_PID_KD,
        POSITION_PID_MAX_ERROR_SUM,
        POSITION_PID_MAX_OUTPUT
      ),
      pitchPID(
        POSITION_PID_KP,
        POSITION_PID_KI,
        POSITION_PID_KD,
        POSITION_PID_MAX_ERROR_SUM,
        POSITION_PID_MAX_OUTPUT
      ) {}

void GimbalChassisRelativeController::initialize() {
    yawPID.reset();
    pitchPID.reset();
}

void GimbalChassisRelativeController::runYawController(float desiredYawAngle) {
    gimbal->setTargetYawAngle(AngleUnit::Radians, desiredYawAngle);

    float positionControllerError = gimbal->getCurrentYawAngleAsContiguousFloat().difference(gimbal->getTargetYawAngle(AngleUnit::Radians));

    yawPID.update(positionControllerError);
    float yawPIDOutput = yawPID.getValue();

    gimbal->setYawMotorOutput(yawPIDOutput);
}

void GimbalChassisRelativeController::runPitchController(float desiredPitchAngle) {
    gimbal->setTargetPitchAngle(AngleUnit::Radians, desiredPitchAngle);

    float positionControllerError = gimbal->getCurrentPitchAngleAsContiguousFloat().difference(gimbal->getTargetPitchAngle(AngleUnit::Radians));

    pitchPID.update(positionControllerError);
    float pitchPIDOutput = pitchPID.getValue();

    pitchPIDOutput += Calculations::computeGravitationalForceOffset(
        GIMBAL_CENTER_OF_GRAVITY_OFFSET_X,
        GIMBAL_CENTER_OF_GRAVITY_OFFSET_Z,
        -gimbal->getCurrentPitchAngleFromCenter(AngleUnit::Radians),
        GRAVITY_COMPENSATION_MAX);

    gimbal->setPitchMotorOutput(pitchPIDOutput);
}

bool GimbalChassisRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal