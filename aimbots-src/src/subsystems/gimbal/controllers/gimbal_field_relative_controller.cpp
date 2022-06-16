#include "gimbal_field_relative_controller.hpp"

namespace src::Gimbal {

GimbalFieldRelativeController::GimbalFieldRelativeController(src::Drivers* drivers, GimbalSubsystem* gimbal)
    : drivers(drivers),
      gimbal(gimbal),
      yawPositionPID(YAW_POSITION_PID_CONFIG),
      pitchPositionPID(PITCH_POSITION_PID_CONFIG) {}

void GimbalFieldRelativeController::initialize() {
    fieldSpaceYawTarget = 0.0f;
}

void GimbalFieldRelativeController::runYawController(AngleUnit unit, float desiredFieldSpaceYawAngle) {
    fieldSpaceYawTarget = (unit == AngleUnit::Degrees) ? desiredFieldSpaceYawAngle : modm::toDegree(desiredFieldSpaceYawAngle);

    // TODO: We might want to limit the yaw angle here. We dont need to
    //       rn, so I'll ignore it, but it's something to consider.

    // Our `desiredFieldSpaceYawAngle` is converted into chassis space
    // and is then set as our gimbal's `targetYawAngle`. This way, we
    // don't even have to convert back to field space to run the PID
    // controller. We just run it the same way we would before.

    // FIXME: Verify that these plus and minus signs work out...
    gimbal->setTargetChassisSpaceYawAngle(AngleUnit::Degrees, fieldSpaceYawTarget - drivers->fieldRelativeInformant.getYaw() + YAW_START_ANGLE);

    float positionControllerError =
        modm::toDegree(gimbal->getCurrentChassisSpaceYawAngleAsContiguousFloat()
                              .difference(gimbal->getTargetChassisSpaceYawAngle(AngleUnit::Radians)));

    float yawPositionPIDOutput =
        yawPositionPID.runController(
            positionControllerError,
            gimbal->getYawMotorRPM() - modm::toDegree(drivers->fieldRelativeInformant.getGz()));

    gimbal->setYawMotorOutput(yawPositionPIDOutput);
}

void GimbalFieldRelativeController::runPitchController(AngleUnit unit, float desiredFieldSpacePitchAngle) {
    gimbal->setTargetChassisSpacePitchAngle(unit, desiredFieldSpacePitchAngle);

    // This gets converted to degrees so that we get a higher error. ig
    // we could also just boost our constants, but this takes minimal
    // calculation and seems simpler. subject to change I suppose...
    float positionControllerError =
        modm::toDegree(gimbal->getCurrentChassisSpacePitchAngleAsContiguousFloat()
                              .difference(gimbal->getTargetChassisSpacePitchAngle(AngleUnit::Radians)));

    float pitchPositionPIDOutput = pitchPositionPID.runController(positionControllerError, gimbal->getPitchMotorRPM());

    gimbal->setPitchMotorOutput(pitchPositionPIDOutput);
}

bool GimbalFieldRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal