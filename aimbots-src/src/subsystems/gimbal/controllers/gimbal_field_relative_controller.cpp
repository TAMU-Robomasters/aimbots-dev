#include "utils/robot_specific_inc.hpp"
#ifdef TOKYO_COMPATIBLE

#include "gimbal_field_relative_controller.hpp"

#define RADPS_TO_RPM 9.549297f

namespace src::Gimbal {

GimbalFieldRelativeController::GimbalFieldRelativeController(src::Drivers* drivers, GimbalSubsystem* gimbalSubsystem)
    : drivers(drivers),
      gimbal(gimbalSubsystem)  //
{
    BuildPositionPIDs();
}

void GimbalFieldRelativeController::initialize() { fieldRelativeYawTarget = 0.0f; }

float fieldRelativeYawTargetDisplay = 0.0f;
float targetYawAxisAngleDisplay = 0.0f;

void GimbalFieldRelativeController::runYawController(AngleUnit unit, float desiredFieldRelativeYawAngle, bool vision) {
    UNUSED(vision);
    fieldRelativeYawTarget =
        (unit == AngleUnit::Radians) ? desiredFieldRelativeYawAngle : modm::toRadian(desiredFieldRelativeYawAngle);

    fieldRelativeYawTargetDisplay = fieldRelativeYawTarget;

    float chassisInducedYawMotionCompensation =
        CHASSIS_VELOCITY_YAW_FEEDFORWARD *
        drivers->kinematicInformant.getIMUAngularVelocity(src::Informants::AngularAxis::YAW_AXIS, AngleUnit::Radians);

    float yawAngularError =
        drivers->kinematicInformant.getCurrentFieldRelativeYawAngleAsContiguousFloat().difference(fieldRelativeYawTarget);

    float chassisRelativeYawTarget = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians) + yawAngularError;

    gimbal->setTargetYawAxisAngle(AngleUnit::Radians, chassisRelativeYawTarget);

    float positionPIDOutput = 0.0f;
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        positionPIDOutput = yawPositionPIDs[i]->runController(
            gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
            gimbal->getYawAxisRPM() + (RADPS_TO_RPM * drivers->kinematicInformant.getIMUAngularVelocity(
                                                          src::Informants::AngularAxis::YAW_AXIS,
                                                          AngleUnit::Radians)));

        gimbal->setDesiredYawMotorOutput(i, positionPIDOutput + chassisInducedYawMotionCompensation);
    }
}

void GimbalFieldRelativeController::runPitchController(AngleUnit unit, float desiredFieldRelativePitchAngle, bool vision) {
    UNUSED(vision);
    fieldRelativeYawTarget =
        (unit == AngleUnit::Radians) ? desiredFieldRelativePitchAngle : modm::toRadian(desiredFieldRelativePitchAngle);

    float pitchAngularError = drivers->kinematicInformant.getCurrentFieldRelativePitchAngleAsContiguousFloat().difference(
        desiredFieldRelativePitchAngle);

    float chassisRelativePitchTarget = gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) + pitchAngularError;

    gimbal->setTargetPitchAxisAngle(unit, chassisRelativePitchTarget);

    float positionPIDOutput = 0.0f;
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        positionPIDOutput = pitchPositionPIDs[i]->runController(
            gimbal->getPitchMotorSetpointError(i, AngleUnit::Radians),
            gimbal->getPitchAxisRPM() + (RADPS_TO_RPM * drivers->kinematicInformant.getIMUAngularVelocity(
                                                            src::Informants::AngularAxis::PITCH_AXIS,
                                                            AngleUnit::Radians)));

        gimbal->setDesiredPitchMotorOutput(i, positionPIDOutput);
    }
}

bool GimbalFieldRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal

#endif  // TOKYO_COMPATIBLE