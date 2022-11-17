#include "gimbal_chassis_relative_controller.hpp"
#ifndef ENGINEER

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
    UNUSED(unit);
    UNUSED(vision);
    gimbal->setTargetChassisRelativeYawAngle(AngleUnit::Degrees, targetChassisRelativeYawAngle);

    float positionControllerError =
        modm::toDegree(
            gimbal->getCurrentChassisRelativeYawAngleAsContiguousFloat()
                .difference(gimbal->getTargetChassisRelativeYawAngle(AngleUnit::Radians)));

    float yawPositionPIDOutput = yawPositionPID.runController(positionControllerError, gimbal->getYawMotorRPM());

    gimbal->setYawMotorOutput(yawPositionPIDOutput);
}

float gravityCompensationDisplay = 0.0f;

void GimbalChassisRelativeController::runPitchController(AngleUnit unit, float targetChassisRelativePitchAngle, bool vision) {
    UNUSED(vision);
    gimbal->setTargetChassisRelativePitchAngle(unit, targetChassisRelativePitchAngle);

    // This gets converted to degrees so that we get a higher error. ig
    // we could also just boost our constants, but this takes minimal
    // calculation and seems simpler. subject to change I suppose...
    float positionControllerError =
        modm::toDegree(
            gimbal->getCurrentChassisRelativePitchAngleAsContiguousFloat()
                .difference(gimbal->getTargetChassisRelativePitchAngle(AngleUnit::Radians)));

    float pitchPositionPIDOutput = pitchPositionPID.runController(positionControllerError, gimbal->getPitchMotorRPM());

    float toHorizonError = gimbal->getCurrentChassisRelativePitchAngleAsContiguousFloat()
                               .difference(modm::toRadian(PITCH_START_ANGLE + HORIZON_OFFSET));

    float gravityCompensation = -cos(toHorizonError) * kGRAVITY;
    gravityCompensationDisplay = gravityCompensation;

    gimbal->setPitchMotorOutput(pitchPositionPIDOutput + gravityCompensation);
}

bool GimbalChassisRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal
#endif