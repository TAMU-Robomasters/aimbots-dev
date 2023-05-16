#include "utils/robot_specific_inc.hpp"
#ifdef TOKYO_COMPATIBLE

#include "gimbal_field_relative_controller.hpp"

#define RADPS_TO_RPM 9.549297f

namespace src::Gimbal {

GimbalFieldRelativeController::GimbalFieldRelativeController(src::Drivers* drivers, GimbalSubsystem* gimbalSubsystem)
    : drivers(drivers),
      gimbal(gimbalSubsystem),
      fieldRelativeYawTarget(0.0f, -M_PI, M_PI),
      fieldRelativePitchTarget(0.0f, -M_PI, M_PI)  //
{
    BuildPositionPIDs();
}

void GimbalFieldRelativeController::initialize() {}

float fieldRelativeYawTargetDisplay = 0.0f;
float targetYawAxisAngleDisplay = 0.0f;

float fieldRelativeYawOutputDisplay = 0.0f;

float feedforwardDisplay = 0.0f;

void GimbalFieldRelativeController::runYawController(bool vision) {
    UNUSED(vision);

    fieldRelativeYawTargetDisplay = this->getTargetYaw(AngleUnit::Degrees);

    // Adds a feedforward term to the yaw controller to compensate for chassis induced yaw motion (based on chassis velocity)
    float chassisInducedYawMotionCompensation =
        -CHASSIS_VELOCITY_YAW_FEEDFORWARD *
        drivers->kinematicInformant.getIMUAngularVelocity(src::Informants::AngularAxis::YAW_AXIS, AngleUnit::Radians) /
        GIMBAL_YAW_GEAR_RATIO;

    feedforwardDisplay = chassisInducedYawMotionCompensation;

    float yawAngularError = drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsContiguousFloat().difference(
        this->getTargetYaw(AngleUnit::Radians));

    // limit error to -180 to 180 because we prefer the gimbal to lag behind the target rather than spin around
    yawAngularError = tap::algorithms::limitVal<float>(yawAngularError, -M_PI, M_PI);

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

    float chassisPitchInGimbalDirection = drivers->kinematicInformant.getChassisPitchInGimbalDirection();

    chassisPitchInGimbalDirectionDisplay = modm::toDegree(chassisPitchInGimbalDirection);

    float pitchAngularError =
        drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsContiguousFloat().difference(
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