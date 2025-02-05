#include "gimbal_field_relative_controller.hpp"

#ifdef GIMBAL_COMPATIBLE
#ifdef GIMBAL_UNTETHERED

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

float pitchOutputVelocityDisplay = 0;

float yawGimbalMotorPositionDisplay = 0.0f;
float yawGimbalMotorPositionTargetDisplay = 0.0f;
float pitchGimbalMotorPositionDisplay = 0.0f;
float pitchGimbalMotorPositionTargetDisplay = 0.0f;

float yawAngleErrorDisplay = 0;
float chassisRelativeYawTargetDisplay = 0;
float kinematicYawAngleDisplay = 0;

float speedTarget = 0.0f;

void GimbalFieldRelativeController::runYawController(
    std::optional<float> velocityLimit) {  // using cascade controller for yaw

    fieldRelativeYawTargetDisplay = this->getTargetYaw(AngleUnit::Degrees);

    kinematicYawAngleDisplay =
        modm::toDegree(drivers->kinematicInformant.fieldRelativeGimbal.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue());

    float yawAngularError = drivers->kinematicInformant.fieldRelativeGimbal.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().minDifference(
        this->getTargetYaw(AngleUnit::Radians));

    // limit error to -180 to 180 because we prefer the gimbal to lag behind the target rather than spin around
    yawAngularError = tap::algorithms::limitVal<float>(yawAngularError, -M_PI, M_PI);

    yawAngleErrorDisplay = yawAngularError;

    float chassisRelativeYawTarget = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians) + yawAngularError;

    chassisRelativeYawTargetDisplay = chassisRelativeYawTarget;

    gimbal->setTargetYawAxisAngle(AngleUnit::Radians, chassisRelativeYawTarget);

    // speedTarget += 1 / 200.0f;
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        yawVelocityFilters[i]->update(RPM_TO_RADPS(gimbal->getYawMotorRPM(i)));

        float fieldRelativeVelocityTarget = yawPositionCascadePIDs[i]->runController(
            gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getYawMotorRPM(i)) + drivers->kinematicInformant.imuData.getIMUAngularVelocity(
                                                          AngularAxis::YAW_AXIS,
                                                          AngleUnit::Radians) /
                                                          GIMBAL_YAW_GEAR_RATIO);

        // float chassisRelativeVelocityTarget = sinf(speedTarget) * 2.0f;

        if (velocityLimit.has_value()) {
            fieldRelativeVelocityTarget =
                tap::algorithms::limitVal<float>(fieldRelativeVelocityTarget, -velocityLimit.value(), velocityLimit.value());
        }

        float chassisRelativeVelocityTarget =
            fieldRelativeVelocityTarget - (drivers->kinematicInformant.imuData.getIMUAngularVelocity(
                                               AngularAxis::YAW_AXIS,
                                               AngleUnit::Radians) /
                                           GIMBAL_YAW_GEAR_RATIO);

        float velocityFeedforward = tap::algorithms::limitVal(
            CHASSIS_VELOCITY_YAW_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
                YAW_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)),
            -GM6020_MAX_OUTPUT,
            GM6020_MAX_OUTPUT);

        // float velocityFeedforward = speedTarget * GM6020_MAX_OUTPUT;  // for tuning feedforward

        chassisRelativeVelocityTargetDisplay = chassisRelativeVelocityTarget;
        chassisRelativeVelocityCurrentDisplay = RPM_TO_RADPS(gimbal->getYawMotorRPM(i));
        // chassisRelativeVelocityCurrentDisplay = yawVelocityFilters[0]->getValue();
        yawGimbalMotorPositionDisplay = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        yawGimbalMotorPositionTargetDisplay = gimbal->getTargetYawAxisAngle(AngleUnit::Radians);

        float velocityControllerOutput = yawVelocityPIDs[i]->runController(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getYawMotorRPM(i)),
            gimbal->getYawMotorTorque(i));

        fieldRelativeVelocityTargetDisplay = fieldRelativeVelocityTarget;
        velocityPIDOutputDisplay = velocityControllerOutput;

        gimbal->setDesiredYawMotorOutput(i, velocityFeedforward + velocityControllerOutput);
    }
}

float pitchAngularErrorDisplay = 0.0f;
float gravComp = 0.0f;

void GimbalFieldRelativeController::runPitchController(std::optional<float> velocityLimit) {
    float pitchAngularError =
        drivers->kinematicInformant.fieldRelativeGimbal.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().minDifference(
            this->getTargetPitch(AngleUnit::Radians));

    pitchAngularErrorDisplay = pitchAngularError;

    float chassisRelativePitchTarget = gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) + pitchAngularError;

    gimbal->setTargetPitchAxisAngle(AngleUnit::Radians, chassisRelativePitchTarget);

    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        float fieldRelativeVelocityTarget = pitchPositionCascadePIDs[i]->runController(
            gimbal->getPitchMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)) + drivers->kinematicInformant.imuData.getIMUAngularVelocity(
                                                            AngularAxis::PITCH_AXIS,
                                                            AngleUnit::Radians) /
                                                            GIMBAL_PITCH_GEAR_RATIO);

        if (velocityLimit.has_value()) {
            fieldRelativeVelocityTarget =
                tap::algorithms::limitVal<float>(fieldRelativeVelocityTarget, -velocityLimit.value(), velocityLimit.value());
        }

        float chassisRelativeVelocityTarget =
            fieldRelativeVelocityTarget -
            (drivers->kinematicInformant.fieldRelativeGimbal.getChassisPitchVelocityInGimbalDirection() / GIMBAL_PITCH_GEAR_RATIO);

        // Gravity compensation should be positive if the gimbal tips forward, negative if it tips backwards
        float gravityCompensationFeedforward =
            kGRAVITY *
            cosf(drivers->kinematicInformant.fieldRelativeGimbal.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().getWrappedValue());

        float velocityFeedforward = gravityCompensationFeedforward +
                                    CHASSIS_VELOCITY_PITCH_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
                                        PITCH_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)) /*+
                                    CHASSIS_LINEAR_ACCELERATION_PITCH_COMPENSATION *
                                        drivers->kinematicInformant.getChassisLinearAccelerationInGimbalDirection()*/
            ;

        velocityFeedforward = tap::algorithms::limitVal(velocityFeedforward, -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT);

        // chassisRelativeVelocityTargetDisplay = chassisRelativeVelocityTarget;
        // chassisRelativeVelocityCurrentDisplay = RPM_TO_RADPS(gimbal->getPitchMotorRPM(i));
        pitchGimbalMotorPositionDisplay = gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians);
        pitchGimbalMotorPositionTargetDisplay = gimbal->getTargetPitchAxisAngle(AngleUnit::Radians);
        velocityFeedforwardOutputDisplay = velocityFeedforward;

        float velocityControllerOutput = pitchVelocityPIDs[i]->runController(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)),
            gimbal->getPitchMotorTorque(i));

        pitchOutputVelocityDisplay = velocityFeedforward + velocityControllerOutput;
        if (gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) < PITCH_AXIS_SOFTSTOP_HIGH + 0.0873 &&
            gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) > PITCH_AXIS_SOFTSTOP_LOW - 0.0873) {
            gimbal->setDesiredPitchMotorOutput(i, velocityFeedforward + velocityControllerOutput);
        } else {
            gimbal->setDesiredPitchMotorOutput(i, 10000 * sgn(kGRAVITY));
        }
    }
}

bool GimbalFieldRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal

#endif  // GIMBAL_COMPATIBLE
#endif  // GIMBAL_UNTETHERED
