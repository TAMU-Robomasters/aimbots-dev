#include "gimbal_chassis_relative_controller.hpp"

#include <utils/robot_specific_inc.hpp>

namespace src::Gimbal {

GimbalChassisRelativeController::GimbalChassisRelativeController(GimbalSubsystem* gimbalSubsystem)
    : gimbal(gimbalSubsystem),
      yawPositionPID(
          YAW_POSITION_PID_KP,
          YAW_POSITION_PID_KI,
          YAW_POSITION_PID_KD,
          YAW_POSITION_PID_MAX_ERROR_SUM,
          POSITION_PID_MAX_OUTPUT),
      pitchPositionPID(
          PITCH_POSITION_PID_KP,
          PITCH_POSITION_PID_KI,
          PITCH_POSITION_PID_KD,
          PITCH_POSITION_PID_MAX_ERROR_SUM,
          POSITION_PID_MAX_OUTPUT) {}

void GimbalChassisRelativeController::initialize() {
    yawPositionPID.reset();
    pitchPositionPID.reset();
}

void GimbalChassisRelativeController::runYawController(AngleUnit unit, float desiredYawAngle) {
    gimbal->setTargetYawAngle(unit, desiredYawAngle);

    float positionControllerError = modm::toDegree(gimbal->getCurrentYawAngleAsContiguousFloat().difference(gimbal->getTargetYawAngle(AngleUnit::Radians)));

    yawPositionPID.update(positionControllerError);
    float yawPositionPIDOutput = yawPositionPID.getValue();

    gimbal->setYawMotorOutput(yawPositionPIDOutput);
}

void GimbalChassisRelativeController::runPitchController(AngleUnit unit, float desiredPitchAngle) {
    desiredPitchAngle = limitPitchAngle((unit == AngleUnit::Degrees) ? desiredPitchAngle : modm::toRadian(desiredPitchAngle));
    gimbal->setTargetPitchAngle(unit, desiredPitchAngle);

    float positionControllerError = modm::toDegree(gimbal->getCurrentPitchAngleAsContiguousFloat().difference(gimbal->getTargetPitchAngle(AngleUnit::Radians)));

    pitchPositionPID.update(positionControllerError);
    float pitchPositionPIDOutput = pitchPositionPID.getValue();

    gimbal->setPitchMotorOutput(pitchPositionPIDOutput);
}

bool GimbalChassisRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal