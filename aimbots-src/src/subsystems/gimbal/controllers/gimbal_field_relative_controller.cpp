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
    BuildPIDControllers();
}

void GimbalFieldRelativeController::initialize() {
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        yawPositionPIDs[i]->pid.reset();
        yawVelocityPIDs[i]->pid.reset();
    }
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        pitchPositionPIDs[i]->pid.reset();
        pitchVelocityPIDs[i]->pid.reset();
    }
}

float fieldRelativeYawTargetDisplay = 0.0f;
float targetYawAxisAngleDisplay = 0.0f;

float fieldRelativeYawOutputDisplay = 0.0f;

float feedforwardDisplay = 0.0f;

float positionPIDOutputDisplay = 0.0f;
float velocityFeedforwardOutputDisplay = 0.0f;
float velocityPIDOutputDisplay = 0.0f;

float chassisRelativeVelocityTargetDisplay = 0.0f;
float chassisRelativeVelocityCurrentDisplay = 0.0f;
float chassisRelativeDerivedVelocityDisplay = 0.0f;
float yawGimbalMotorPositionDisplay = 0.0f;
float yawGimbalMotorPositionTargetDisplay = 0.0f;

float speedTarget = 0.0f;

void GimbalFieldRelativeController::runYawController(bool vision) {  // using cascade controller for yaw
    UNUSED(vision);

    fieldRelativeYawTargetDisplay = this->getTargetYaw(AngleUnit::Degrees);

    float yawAngularError = drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsContiguousFloat().difference(
        this->getTargetYaw(AngleUnit::Radians));

    // limit error to -180 to 180 because we prefer the gimbal to lag behind the target rather than spin around
    yawAngularError = tap::algorithms::limitVal<float>(yawAngularError, -M_PI, M_PI);

    float chassisRelativeYawTarget = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians) + yawAngularError;

    gimbal->setTargetYawAxisAngle(AngleUnit::Radians, chassisRelativeYawTarget);

    // speedTarget += 1 / 200.0f;
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        yawVelocityFilters[i]->update(RPM_TO_RADPS(gimbal->getYawMotorRPM(i)));

        float positionPIDOutput = yawPositionCascadePIDs[i]->runController(
            gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getYawMotorRPM(i)) + drivers->kinematicInformant.getIMUAngularVelocity(
                                                          src::Informants::AngularAxis::YAW_AXIS,
                                                          AngleUnit::Radians) /
                                                          GIMBAL_YAW_GEAR_RATIO);

        // float chassisRelativeVelocityTarget = sinf(speedTarget) * 2.0f;

        float chassisRelativeVelocityTarget =
            positionPIDOutput -
            (drivers->kinematicInformant.getIMUAngularVelocity(src::Informants::AngularAxis::YAW_AXIS, AngleUnit::Radians)) /
                GIMBAL_YAW_GEAR_RATIO;

        float velocityFeedforward = CHASSIS_VELOCITY_YAW_FEEDFORWARD * chassisRelativeVelocityTarget;

        chassisRelativeVelocityTargetDisplay = chassisRelativeVelocityTarget;
        chassisRelativeVelocityCurrentDisplay = RPM_TO_RADPS(gimbal->getYawMotorRPM(i));
        yawGimbalMotorPositionDisplay = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        yawGimbalMotorPositionTargetDisplay = gimbal->getTargetYawAxisAngle(AngleUnit::Radians);

        float velocityControllerOutput = yawVelocityPIDs[i]->runController(
            chassisRelativeVelocityTarget - yawVelocityFilters[i]->getValue(),
            gimbal->getYawMotorTorque(i));

        positionPIDOutputDisplay = positionPIDOutput;
        velocityFeedforwardOutputDisplay = velocityFeedforward;
        velocityPIDOutputDisplay = velocityControllerOutput;

        gimbal->setDesiredYawMotorOutput(i, velocityFeedforward + velocityControllerOutput);
    }
}

void GimbalFieldRelativeController::runPitchController(bool vision) {
    UNUSED(vision);

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