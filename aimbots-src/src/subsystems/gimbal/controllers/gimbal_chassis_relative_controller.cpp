#include "gimbal_chassis_relative_controller.hpp"

#include <utils/robot_specific_inc.hpp>

namespace src::Gimbal {

static constexpr int THING = 1;

static inline float limitPitchAngle(float angle) {
    if constexpr (PITCH_HARDSTOP_LOW < PITCH_HARDSTOP_HIGH) {
        return tap::algorithms::limitVal(angle, PITCH_HARDSTOP_LOW, PITCH_HARDSTOP_HIGH);
    } else if constexpr (constAbs(PITCH_HARDSTOP_HIGH - PITCH_HARDSTOP_LOW) > 180.0f) {
        // FIXME: Implement this check
        return 0.0f;
    } else {
        return tap::algorithms::limitVal(angle, PITCH_HARDSTOP_HIGH, PITCH_HARDSTOP_LOW);
    }
}

GimbalChassisRelativeController::GimbalChassisRelativeController(GimbalSubsystem* gimbalSubsystem)
    : gimbal(gimbalSubsystem),
      yawPositionPID(
          YAW_POSITION_PID_KP,
          YAW_POSITION_PID_KI,
          YAW_POSITION_PID_KD,
          YAW_POSITION_PID_MAX_ERROR_SUM,
          POSITION_PID_MAX_OUTPUT,
          YAW_POSITION_PID_Q_DERIVATIVE_KALMAN,
          YAW_POSITION_PID_R_DERIVATIVE_KALMAN,
          YAW_POSITION_PID_Q_PROPORTIONAL_KALMAN,
          YAW_POSITION_PID_R_PROPORTIONAL_KALMAN),
      pitchPositionPID(
          PITCH_POSITION_PID_KP,
          PITCH_POSITION_PID_KI,
          PITCH_POSITION_PID_KD,
          PITCH_POSITION_PID_MAX_ERROR_SUM,
          POSITION_PID_MAX_OUTPUT,
          PITCH_POSITION_PID_Q_DERIVATIVE_KALMAN,
          PITCH_POSITION_PID_R_DERIVATIVE_KALMAN,
          PITCH_POSITION_PID_Q_PROPORTIONAL_KALMAN,
          PITCH_POSITION_PID_R_PROPORTIONAL_KALMAN) {}

void GimbalChassisRelativeController::initialize() {
    yawPositionPID.pid.reset();
    pitchPositionPID.pid.reset();
}

void GimbalChassisRelativeController::runYawController(AngleUnit unit, float desiredYawAngle) {
    gimbal->setTargetYawAngle(unit, desiredYawAngle);

    float positionControllerError = modm::toDegree(gimbal->getCurrentYawAngleAsContiguousFloat().difference(gimbal->getTargetYawAngle(AngleUnit::Radians)));

    float yawPositionPIDOutput = yawPositionPID.runController(positionControllerError, gimbal->getYawMotorRPM());

    gimbal->setYawMotorOutput(yawPositionPIDOutput);
}

void GimbalChassisRelativeController::runPitchController(AngleUnit unit, float desiredPitchAngle) {
    desiredPitchAngle = limitPitchAngle((unit == AngleUnit::Degrees) ? desiredPitchAngle : modm::toRadian(desiredPitchAngle));
    gimbal->setTargetPitchAngle(unit, desiredPitchAngle);

    float positionControllerError = modm::toDegree(gimbal->getCurrentPitchAngleAsContiguousFloat().difference(gimbal->getTargetPitchAngle(AngleUnit::Radians)));

    float pitchPositionPIDOutput = pitchPositionPID.runController(positionControllerError, gimbal->getPitchMotorRPM());

    gimbal->setPitchMotorOutput(pitchPositionPIDOutput);
}

bool GimbalChassisRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal