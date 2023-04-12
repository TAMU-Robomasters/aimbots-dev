#include "gimbal_chassis_relative_controller.hpp"

#include <utils/robot_specific_inc.hpp>

namespace src::Gimbal {

GimbalChassisRelativeController::GimbalChassisRelativeController(GimbalSubsystem* gimbalSubsystem)
    : gimbal(gimbalSubsystem),
      yawPositionPID(YAW_POSITION_PID_CONFIG),
      pitchPositionPID(PITCH_POSITION_PID_CONFIG) {}

void GimbalChassisRelativeController::initialize() {
    yawPositionPID.pid.reset();
    pitchPositionPID.pid.reset();
}

void GimbalChassisRelativeController::runYawController(AngleUnit unit, float targetChassisRelativeYawAngle, bool vision) {
    UNUSED(vision);
    gimbal->setTargetYawAngle(unit, targetChassisRelativeYawAngle);

    float positionControllerError = modm::toDegree(
        gimbal->getCurrentYawMotorAngleAsContiguousFloat().difference(gimbal->getTargetYawMotorAngle(AngleUnit::Radians)));

    float yawPositionPIDOutput = yawPositionPID.runController(positionControllerError, gimbal->getYawMotorRPM());

    gimbal->setYawMotorOutput(yawPositionPIDOutput);
}

float gravityCompensationDisplay = 0.0f;

void GimbalChassisRelativeController::runPitchController(
    AngleUnit unit,
    float targetChassisRelativePitchAngle,
    bool vision) {
    UNUSED(vision);
    gimbal->setTargetPitchAngle(unit, targetChassisRelativePitchAngle);

    // This gets converted to degrees so that we get a higher error. ig
    // we could also just boost our constants, but this takes minimal
    // calculation and seems simpler. subject to change I suppose...
    float positionControllerError = modm::toDegree(
        gimbal->getCurrentPitchMotorAngleAsContiguousFloat().difference(gimbal->getTargetPitchAngle(AngleUnit::Radians)));

    float pitchPositionPIDOutput = pitchPositionPID.runController(positionControllerError, gimbal->getPitchMotorRPM());

    float toHorizonError =
        gimbal->getCurrentPitchMotorAngleAsContiguousFloat().difference(modm::toRadian(PITCH_OFFSET_ANGLE + HORIZON_OFFSET));

    float gravityCompensation = -cos(toHorizonError) * kGRAVITY;
    gravityCompensationDisplay = gravityCompensation;

    gimbal->setPitchMotorOutput(pitchPositionPIDOutput + gravityCompensation);
}

bool GimbalChassisRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal