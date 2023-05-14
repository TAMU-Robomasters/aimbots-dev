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

float fieldRelativeYawOutputDisplay = 0.0f;

float feedforwardDisplay = 0.0f;

void GimbalFieldRelativeController::runYawController(bool vision) {
    UNUSED(vision);

    fieldRelativeYawTargetDisplay = this->getTargetYaw(AngleUnit::Degrees);

    float chassisInducedYawMotionCompensation =
        -CHASSIS_VELOCITY_YAW_FEEDFORWARD *
        drivers->kinematicInformant.getIMUAngularVelocity(src::Informants::AngularAxis::YAW_AXIS, AngleUnit::Radians);

    feedforwardDisplay = chassisInducedYawMotionCompensation;

    // scale rad/s to 16'000 power

    float yawAngularError = drivers->kinematicInformant.getCurrentFieldRelativeYawAngleAsContiguousFloat().difference(
        this->getTargetYaw(AngleUnit::Radians));

    float chassisRelativeYawTarget = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians) + yawAngularError;

    gimbal->setTargetYawAxisAngle(AngleUnit::Radians, chassisRelativeYawTarget);

    float positionPIDOutput = 0.0f;
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        positionPIDOutput = yawPositionPIDs[i]->runController(
            gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getYawMotorRPM(i)) + drivers->kinematicInformant.getIMUAngularVelocity(
                                                          src::Informants::AngularAxis::YAW_AXIS,
                                                          AngleUnit::Radians) /
                                                          GIMBAL_YAW_GEAR_RATIO);

        fieldRelativeYawOutputDisplay = positionPIDOutput;

        gimbal->setDesiredYawMotorOutput(i, positionPIDOutput + chassisInducedYawMotionCompensation);
    }
}

float chassisPitchInGimbalDirectionDisplay = 0.0f;

void GimbalFieldRelativeController::runPitchController(bool vision) {
    UNUSED(vision);

    float chassisYaw =
        drivers->kinematicInformant.getChassisIMUAngle(src::Informants::AngularAxis::YAW_AXIS, AngleUnit::Radians);

    float sinYaw = sinf(chassisYaw);
    float cosYaw = cosf(chassisYaw);

    float chassisRoll =
        drivers->kinematicInformant.getChassisIMUAngle(src::Informants::AngularAxis::ROLL_AXIS, AngleUnit::Radians);

    float sinRoll = sinf(chassisRoll);
    float cosRoll = cosf(chassisRoll);

    float chassisPitch =
        drivers->kinematicInformant.getChassisIMUAngle(src::Informants::AngularAxis::PITCH_AXIS, AngleUnit::Radians);

    float sinPitch = sinf(chassisPitch);
    float cosPitch = cosf(chassisPitch);

    float chassisPitchInGimbalDirection = atan2f(
        cosYaw * sinPitch + sinYaw * sinRoll,
        sqrtf(sinYaw * sinYaw * cosRoll * cosRoll + cosYaw * cosYaw * cosPitch * cosPitch));

    chassisPitchInGimbalDirectionDisplay = modm::toDegree(chassisPitchInGimbalDirection);

    float pitchAngularError = drivers->kinematicInformant.getCurrentFieldRelativePitchAngleAsContiguousFloat().difference(
        this->getTargetPitch(AngleUnit::Radians));

    float chassisRelativePitchTarget = gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) + pitchAngularError;

    gimbal->setTargetPitchAxisAngle(AngleUnit::Radians, chassisRelativePitchTarget);

    float positionPIDOutput = 0.0f;
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        positionPIDOutput = pitchPositionPIDs[i]->runController(
            gimbal->getPitchMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)) + drivers->kinematicInformant.getIMUAngularVelocity(
                                                            src::Informants::AngularAxis::PITCH_AXIS,
                                                            AngleUnit::Radians) /
                                                            GIMBAL_PITCH_GEAR_RATIO);

        gimbal->setDesiredPitchMotorOutput(i, positionPIDOutput);
    }
}

bool GimbalFieldRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal

#endif  // TOKYO_COMPATIBLE