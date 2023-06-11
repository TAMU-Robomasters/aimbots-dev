#include "utils/robot_specific_inc.hpp"

#ifdef GIMBAL_UNTETHERED

#include "gimbal_field_relative_controller.hpp"

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

float fieldRelativeVelocityTargetDisplay = 0.0f;
float velocityFeedforwardOutputDisplay = 0.0f;
float velocityPIDOutputDisplay = 0.0f;

float chassisRelativeVelocityTargetDisplay = 0.0f;
float chassisRelativeVelocityCurrentDisplay = 0.0f;
float chassisRelativeDerivedVelocityDisplay = 0.0f;
float yawGimbalMotorPositionDisplay = 0.0f;
float yawGimbalMotorPositionTargetDisplay = 0.0f;

float speedTarget = 0.0f;

void GimbalFieldRelativeController::runYawController(
    std::optional<float> velocityLimit) {  // using cascade controller for yaw

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

        float fieldRelativeVelocityTarget = yawPositionCascadePIDs[i]->runController(
            gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getYawMotorRPM(i)) + drivers->kinematicInformant.getIMUAngularVelocity(
                                                          src::Informants::AngularAxis::YAW_AXIS,
                                                          AngleUnit::Radians) /
                                                          GIMBAL_YAW_GEAR_RATIO);

        // float chassisRelativeVelocityTarget = sinf(speedTarget) * 2.0f;

        if (velocityLimit.has_value()) {
            fieldRelativeVelocityTarget =
                tap::algorithms::limitVal<float>(fieldRelativeVelocityTarget, -velocityLimit.value(), velocityLimit.value());
        }

        float chassisRelativeVelocityTarget =
            fieldRelativeVelocityTarget -
            (drivers->kinematicInformant.getIMUAngularVelocity(src::Informants::AngularAxis::YAW_AXIS, AngleUnit::Radians) /
             GIMBAL_YAW_GEAR_RATIO);

        float velocityFeedforward = tap::algorithms::limitVal(
            CHASSIS_VELOCITY_YAW_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
                GM6020_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)),
            -GM6020_MAX_OUTPUT,
            GM6020_MAX_OUTPUT);

        // float velocityFeedforward = speedTarget * GM6020_MAX_OUTPUT; // for tuning feedforward

        chassisRelativeVelocityTargetDisplay = chassisRelativeVelocityTarget;
        // chassisRelativeVelocityCurrentDisplay = RPM_TO_RADPS(gimbal->getYawMotorRPM(i));
        chassisRelativeVelocityCurrentDisplay = yawVelocityFilters[i]->getValue();
        yawGimbalMotorPositionDisplay = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        yawGimbalMotorPositionTargetDisplay = gimbal->getTargetYawAxisAngle(AngleUnit::Radians);

        float velocityControllerOutput = yawVelocityPIDs[i]->runController(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getYawMotorRPM(i)),
            gimbal->getYawMotorTorque(i));

        fieldRelativeVelocityTargetDisplay = fieldRelativeVelocityTarget;
        velocityFeedforwardOutputDisplay = velocityFeedforward;
        velocityPIDOutputDisplay = velocityControllerOutput;

        gimbal->setDesiredYawMotorOutput(i, velocityFeedforward + velocityControllerOutput);
    }
}

void GimbalFieldRelativeController::runPitchController() {
    float pitchAngularError =
        drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsContiguousFloat().difference(
            this->getTargetPitch(AngleUnit::Radians));

    float chassisRelativePitchTarget = gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) + pitchAngularError;

    gimbal->setTargetPitchAxisAngle(AngleUnit::Radians, chassisRelativePitchTarget);

    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        float fieldRelativeVelocityTarget = pitchPositionCascadePIDs[i]->runController(
            gimbal->getPitchMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)) + drivers->kinematicInformant.getIMUAngularVelocity(
                                                            src::Informants::AngularAxis::PITCH_AXIS,
                                                            AngleUnit::Radians) /
                                                            GIMBAL_PITCH_GEAR_RATIO);

        float chassisRelativeVelocityTarget =
            fieldRelativeVelocityTarget -
            (drivers->kinematicInformant.getChassisPitchVelocityInGimbalDirection() / GIMBAL_YAW_GEAR_RATIO);

        float velocityFeedforward = 0;
        // float velocityFeedforward = CHASSIS_VELOCITY_PITCH_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
        //                                 GM6020_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)) +
        //                             CHASSIS_LINEAR_ACCELERATION_PITCH_COMPENSATION *
        //                                 drivers->kinematicInformant.getChassisLinearAccelerationInGimbalDirection();

        // velocityFeedforward = tap::algorithms::limitVal(velocityFeedforward, -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT);

        float velocityControllerOutput = pitchVelocityPIDs[i]->runController(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)),
            gimbal->getPitchMotorTorque(i));

        gimbal->setDesiredPitchMotorOutput(i, velocityFeedforward + velocityControllerOutput);
    }
}

bool GimbalFieldRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal

#endif  // GIMBAL_UNTETHERED