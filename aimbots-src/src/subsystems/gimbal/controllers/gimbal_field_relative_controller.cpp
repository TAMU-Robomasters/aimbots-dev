#include "utils/robot_specific_inc.hpp"
#ifdef TOKYO_COMPATIBLE

#include "gimbal_field_relative_controller.hpp"

#define RADPS_TO_RPM 9.549297f

namespace src::Gimbal {

GimbalFieldRelativeController::GimbalFieldRelativeController(src::Drivers* drivers, GimbalSubsystem* gimbal)
    : drivers(drivers),
      gimbal(gimbal),
      yawPositionPID(YAW_POSITION_PID_CONFIG),
      pitchPositionPID(PITCH_POSITION_PID_CONFIG),
      yawVisionPID(YAW_VISION_PID_CONFIG),
      pitchVisionPID(PITCH_VISION_PID_CONFIG) {}

void GimbalFieldRelativeController::initialize() {
    fieldRelativeYawTarget = 0.0f;
}

float fieldRelativeYawTargetDisplay = 0.0f;
float targetChassisRelativeYawAngleDisplay = 0.0f;

void GimbalFieldRelativeController::runYawController(AngleUnit unit, float desiredFieldRelativeYawAngle, bool vision) {
    fieldRelativeYawTarget = (unit == AngleUnit::Degrees) ? desiredFieldRelativeYawAngle : modm::toDegree(desiredFieldRelativeYawAngle);
    // converts to degrees if necessary
    fieldRelativeYawTargetDisplay = fieldRelativeYawTarget;

    float chassisInducedYawMotionCompensation = CHASSIS_VELOCITY_YAW_FEEDFORWARD * drivers->fieldRelativeInformant.getGz();

    float positionControllerError = modm::toDegree(gimbal->getCurrentFieldRelativeYawAngleAsContiguousFloat().difference(modm::toRadian(fieldRelativeYawTarget)));
    float yawPositionPIDOutput = 0.0f;

    if (!vision) {
        yawPositionPIDOutput = yawPositionPID.runController(positionControllerError, gimbal->getYawMotorRPM() + (RADPS_TO_RPM * drivers->fieldRelativeInformant.getGz()));
    } else {
        yawPositionPIDOutput = yawVisionPID.runController(positionControllerError, gimbal->getYawMotorRPM() + (RADPS_TO_RPM * drivers->fieldRelativeInformant.getGz()));
    }  // kD tuned for RPM, so we'll convert to RPM

    gimbal->setYawMotorOutput(yawPositionPIDOutput + chassisInducedYawMotionCompensation);
}

void GimbalFieldRelativeController::runPitchController(AngleUnit unit, float desiredFieldRelativePitchAngle, bool vision) {
    gimbal->setTargetChassisRelativePitchAngle(unit, desiredFieldRelativePitchAngle);

    float positionControllerError = modm::toDegree(gimbal->getCurrentChassisRelativePitchAngleAsContiguousFloat().difference(gimbal->getTargetChassisRelativePitchAngle(AngleUnit::Radians)));
    float pitchPositionPIDOutput = 0.0f;

    if (!vision) {
        pitchPositionPIDOutput = pitchPositionPID.runController(positionControllerError, gimbal->getPitchMotorRPM());
    } else {
        pitchPositionPIDOutput = pitchVisionPID.runController(positionControllerError, gimbal->getPitchMotorRPM());
    }

    gimbal->setPitchMotorOutput(pitchPositionPIDOutput);
}

bool GimbalFieldRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal

#endif  // TOKYO_COMPATIBLE