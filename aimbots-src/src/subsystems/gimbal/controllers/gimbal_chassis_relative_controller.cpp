#include "gimbal_chassis_relative_controller.hpp"

#include <utils/robot_specific_inc.hpp>

namespace src::Gimbal {

GimbalChassisRelativeController::GimbalChassisRelativeController(GimbalSubsystem* gimbalSubsystem)
    : gimbal(gimbalSubsystem),
      yawPositionPID(
          POSITION_PID_KP,
          POSITION_PID_KI,
          POSITION_PID_KD,
          POSITION_PID_MAX_ERROR_SUM,
          POSITION_PID_MAX_OUTPUT),
      pitchPositionPID(
          POSITION_PID_KP,
          POSITION_PID_KI,
          POSITION_PID_KD,
          POSITION_PID_MAX_ERROR_SUM,
          POSITION_PID_MAX_OUTPUT) {}

void GimbalChassisRelativeController::initialize() {
    yawPositionPID.reset();
    pitchPositionPID.reset();
}

static float outputPitch = 0.0f;
static float outputYaw = 0.0f;

void GimbalChassisRelativeController::runYawController(float desiredYawAngle) {
    gimbal->setTargetYawAngle(AngleUnit::Degrees, desiredYawAngle);

    float positionControllerError = modm::toDegree(gimbal->getCurrentYawAngleAsContiguousFloat().difference(gimbal->getTargetYawAngle(AngleUnit::Degrees)));

    yawPositionPID.update(positionControllerError);
    float yawPositionPIDOutput = yawPositionPID.getValue();

    outputYaw = yawPositionPIDOutput;
    gimbal->setYawMotorOutput(yawPositionPIDOutput);
}

void GimbalChassisRelativeController::runPitchController(float desiredPitchAngle) {
    // limitVal looks backwards, but that's because PITCH_HARDSTOP_HIGH is a lower
    // encoder angle than PITCH_HARDSTOP_LOW because of the way the motor is mounted.

    // FIXME: This might not work work for all robots, so verify this is valid before
    //        running the code.
    desiredPitchAngle = tap::algorithms::limitVal(desiredPitchAngle, PITCH_HARDSTOP_HIGH, PITCH_HARDSTOP_LOW);
    gimbal->setTargetPitchAngle(AngleUnit::Degrees, desiredPitchAngle);

    float positionControllerError = modm::toDegree(gimbal->getCurrentPitchAngleAsContiguousFloat().difference(gimbal->getTargetPitchAngle(AngleUnit::Radians)));

    pitchPositionPID.update(positionControllerError);
    float pitchPositionPIDOutput = pitchPositionPID.getValue();

    outputPitch = pitchPositionPIDOutput;

    gimbal->setPitchMotorOutput(pitchPositionPIDOutput);
}

bool GimbalChassisRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal