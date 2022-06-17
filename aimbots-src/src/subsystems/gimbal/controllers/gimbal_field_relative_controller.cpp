#include "gimbal_field_relative_controller.hpp"

#define RADPS_TO_RPM 9.549297f

namespace src::Gimbal {

GimbalFieldRelativeController::GimbalFieldRelativeController(src::Drivers* drivers, GimbalSubsystem* gimbal)
    : drivers(drivers),
      gimbal(gimbal),
      yawPositionPID(YAW_POSITION_PID_CONFIG),
      pitchPositionPID(PITCH_POSITION_PID_CONFIG) {}

void GimbalFieldRelativeController::initialize() {
    fieldRelativeYawTarget = 0.0f;
}

void GimbalFieldRelativeController::runYawController(AngleUnit unit, float desiredFieldRelativeYawAngle) {
    fieldRelativeYawTarget = (unit == AngleUnit::Degrees) ? desiredFieldRelativeYawAngle : modm::toDegree(desiredFieldRelativeYawAngle);

    // TODO: We might want to limit the yaw angle here. We dont need to
    //       rn, so I'll ignore it, but it's something to consider.

    // Our `desiredFieldRelativeYawAngle` is converted into chassis space
    // and is then set as our gimbal's `targetYawAngle`. This way, we
    // don't even have to convert back to field space to run the PID
    // controller. We just run it the same way we would before.

    // !FIXME: Verify that these plus and minus signs work out...
    gimbal->setTargetChassisRelativeYawAngle(AngleUnit::Degrees, fieldRelativeYawTarget - drivers->fieldRelativeInformant.getYaw() + YAW_START_ANGLE);

    float positionControllerError = modm::toDegree(gimbal->getCurrentChassisRelativeYawAngleAsContiguousFloat().difference(gimbal->getTargetChassisRelativeYawAngle(AngleUnit::Radians)));

    float yawPositionPIDOutput = yawPositionPID.runController(positionControllerError, gimbal->getYawMotorRPM() - (RADPS_TO_RPM * drivers->fieldRelativeInformant.getGz()));
    // kD tuned for RPM, so we'll convert to RPM

    gimbal->setYawMotorOutput(yawPositionPIDOutput);
}

void GimbalFieldRelativeController::runPitchController(AngleUnit unit, float desiredFieldRelativePitchAngle) {
    gimbal->setTargetChassisRelativePitchAngle(unit, desiredFieldRelativePitchAngle);

    // This gets converted to degrees so that we get a higher error. ig
    // we could also just boost our constants, but this takes minimal
    // calculation and seems simpler. subject to change I suppose...
    float positionControllerError =
        modm::toDegree(gimbal->getCurrentChassisRelativePitchAngleAsContiguousFloat()
                           .difference(gimbal->getTargetChassisRelativePitchAngle(AngleUnit::Radians)));

    float pitchPositionPIDOutput = pitchPositionPID.runController(positionControllerError, gimbal->getPitchMotorRPM());

    gimbal->setPitchMotorOutput(pitchPositionPIDOutput);
}

bool GimbalFieldRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal