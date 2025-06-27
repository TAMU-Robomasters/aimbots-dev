#include "gimbal_field_relative_controller.hpp"

#ifdef GIMBAL_COMPATIBLE
#ifdef GIMBAL_UNTETHERED

namespace src::Gimbal {

GimbalFieldRelativeController::GimbalFieldRelativeController(src::Drivers* drivers, GimbalSubsystem* gimbalSubsystem, bool forCV)
    : drivers(drivers),
      gimbal(gimbalSubsystem),
      forCV(forCV),
      fieldRelativeYawTarget(0.0f, -M_PI, M_PI),
      fieldRelativePitchTarget(0.0f, -M_PI, M_PI)  //
{
    BuildPIDControllers();
}

void GimbalFieldRelativeController::initialize() {
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        yawPositionPIDs[i]->pid.reset(); //? should this be yawPositionCascadePIDs
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

float fieldRelativeYawVelocityTargetDisplay = 0.0f;
float fieldRelativePitchVelocityTargetDisplay = 0.0f;
float pitchVelocityFeedforwardOutputDisplay = 0.0f;
float yawVelocityFeedforwardOutputDisplay = 0.0f;
float velocityPIDOutputDisplay = 0.0f;

float chassisYawRelativeVelocityTargetDisplay = 0.0f;
float chassisYawRelativeVelocityCurrentDisplay = 0.0f;
float chassisYawRelativeDerivedVelocityDisplay = 0.0f;

float chassisPitchRelativeVelocityTargetDisplay = 0.0f;
float chassisPitchRelativeVelocityCurrentDisplay = 0.0f;
float chassisPitchRelativeDerivedVelocityDisplay = 0.0f;

float chassisRelativeAccelerationTargetDisplay = 0.0f;
float fieldRelativeAccelerationTargetDisplay = 0.0f;
float chassisIMUAccelerationDisplay = 0.0f;

float pitchOutputVelocityDisplay = 0;

float yawGimbalMotorPositionDisplay = 0.0f;
float yawGimbalMotorPositionTargetDisplay = 0.0f;
float pitchGimbalMotorPositionDisplay = 0.0f;
float pitchGimbalMotorPositionTargetDisplay = 0.0f;

float yawAngleErrorDisplay = 0;
float chassisRelativeYawTargetDisplay = 0;
float kinematicYawAngleDisplay = 0;
float kinematicPitchAngleDisplay = 0;

float speedTarget = 0.0f;

// for tunning PID system through Ozone
float yawVelocityPDebug = 0.0f;
float yawVelocityIDebug = 0.0f;
float yawVelocityDDebug = 0.0f;
bool updateYawVelocityPIDsDebug = false;

float yawPositionPDebug = 0.0f;
float yawPositionIDebug = 0.0f;
float yawPositionDDebug = 0.0f;
bool updateYawPositionPIDsDebug = false;

float pitchPositionPDebug = 0.0f;
float pitchPositionIDebug = 0.0f;
float pitchPositionDDebug = 0.0f;
bool updatePitchPositionPIDsDebug = false;

float pitchVelocityPDebug = 0.0f;
float pitchVelocityIDebug= 0.0f;
float pitchVelocityDDebug= 0.0f;
bool updatePitchVelocityPIDsDebug = false;

float kYawBallisticVelocityDebug = 0.0f;
float kYawBallisticAccelerationDebug = 0.0f;
bool updateYawBallisticFeedforwardDebug = false;

float kPitchBallisticVelocityDebug = 0.0f;
float kPitchBallisticAccelerationDebug = 0.0f;
bool updatePitchBallisticFeedforwardDebug = false;


//TODO:should I limit feedforward plus PID

void GimbalFieldRelativeController::runYawController(
    std::optional<float> velocityLimit) {  // using cascade controller for yaw

    fieldRelativeYawTargetDisplay = this->getTargetYaw(AngleUnit::Degrees);

    kinematicYawAngleDisplay =
        modm::toDegree(drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue());

    float yawAngularError = drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().minDifference(
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

        if (updateYawPositionPIDsDebug) {
            yawPositionCascadePIDs[i]->pid.setP(yawPositionPDebug);
            yawPositionCascadePIDs[i]->pid.setI(yawPositionIDebug);
            yawPositionCascadePIDs[i]->pid.setD(yawPositionDDebug);
            yawPositionCascadePIDs[i]->pid.reset();
            updateYawPositionPIDsDebug = false;
        }
        if (forCV) {
            yawOuterLoopCounter++;
            if (yawOuterLoopCounter % 10 == 0) {
                fieldRelativeVelocityTarget = yawPositionCascadePIDs[i]->runController(
                gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
                RPM_TO_RADPS(gimbal->getYawMotorRPM(i)) + drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                                            src::Informants::AngularAxis::YAW_AXIS,
                                                            AngleUnit::Radians) /
                                                            GIMBAL_YAW_GEAR_RATIO);
            }
        } else {
            fieldRelativeVelocityTarget = yawPositionCascadePIDs[i]->runController(
            gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getYawMotorRPM(i)) + drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                                            src::Informants::AngularAxis::YAW_AXIS,
                                                            AngleUnit::Radians) /
                                                            GIMBAL_YAW_GEAR_RATIO);
        }
        

        // float chassisRelativeVelocityTarget = sinf(speedTarget) * 2.0f;

        if (velocityLimit.has_value()) {
            fieldRelativeVelocityTarget =
                tap::algorithms::limitVal<float>(fieldRelativeVelocityTarget, -velocityLimit.value(), velocityLimit.value());
        }

        float chassisRelativeVelocityTarget =
            fieldRelativeVelocityTarget - (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                               src::Informants::AngularAxis::YAW_AXIS,
                                               AngleUnit::Radians) /
                                           GIMBAL_YAW_GEAR_RATIO);

        float velocityFeedforward = tap::algorithms::limitVal(
            CHASSIS_VELOCITY_YAW_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
                YAW_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)),
            -GM6020_MAX_OUTPUT,
            GM6020_MAX_OUTPUT);

        if (updateYawBallisticFeedforwardDebug) {
            kYawBallisticVelocity = kYawBallisticVelocityDebug;
            updateYawBallisticFeedforwardDebug = false;
        }
        float chassisRelativeBallisticVelocityTarget = this->getTargetVelocityYaw(AngleUnit::Radians) - (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                               src::Informants::AngularAxis::YAW_AXIS,
                                               AngleUnit::Radians) /
                                           GIMBAL_YAW_GEAR_RATIO);

        float ballisticFeedforward = kYawBallisticVelocity * chassisRelativeBallisticVelocityTarget;

        // float velocityFeedforward = speedTarget * GM6020_MAX_OUTPUT;  // for tuning feedforward

        chassisYawRelativeVelocityTargetDisplay = chassisRelativeVelocityTarget;
        chassisYawRelativeVelocityCurrentDisplay = modm::toDegree(RPM_TO_RADPS(gimbal->getYawMotorRPM(i)));
        // chassisRelativeVelocityCurrentDisplay = yawVelocityFilters[0]->getValue();
        yawGimbalMotorPositionDisplay = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        yawGimbalMotorPositionTargetDisplay = gimbal->getTargetYawAxisAngle(AngleUnit::Radians);

        float velocityControllerOutput = yawVelocityPIDs[i]->runController(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getYawMotorRPM(i)),
            gimbal->getYawMotorTorque(i));

        fieldRelativeYawVelocityTargetDisplay = fieldRelativeVelocityTarget;
        velocityPIDOutputDisplay = velocityControllerOutput;

        if (forCV) gimbal->setDesiredYawMotorOutput(i, /*velocityFeedforward +*/ velocityControllerOutput + ballisticFeedforward);
        else gimbal->setDesiredYawMotorOutput(i, velocityFeedforward + velocityControllerOutput);
    }
}

// for PID testing
void GimbalFieldRelativeController::runYawVelocityController(
    std::optional<float> velocityLimit) {

    fieldRelativeYawVelocityTargetDisplay = this->getTargetVelocityYaw(AngleUnit::Degrees);
    
    kinematicYawAngleDisplay =
        modm::toDegree(drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue());

    // speedTarget += 1 / 200.0f;
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        yawVelocityFilters[i]->update(modm::toDegree(RPM_TO_RADPS(gimbal->getYawMotorRPM(i))));

        float fieldRelativeVelocityTarget = this->getTargetVelocityYaw(AngleUnit::Radians);

        if (velocityLimit.has_value()) {
            fieldRelativeVelocityTarget =
                tap::algorithms::limitVal<float>(fieldRelativeVelocityTarget, -velocityLimit.value(), velocityLimit.value());
        }

        float chassisRelativeVelocityTarget =
            fieldRelativeVelocityTarget - (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                               src::Informants::AngularAxis::YAW_AXIS,
                                               AngleUnit::Radians) /
                                           GIMBAL_YAW_GEAR_RATIO);


        float velocityFeedforward = tap::algorithms::limitVal(
            CHASSIS_VELOCITY_YAW_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
                YAW_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)),
            -GM6020_MAX_OUTPUT,
            GM6020_MAX_OUTPUT);

        if (updateYawBallisticFeedforwardDebug) {
            kYawBallisticVelocity = kYawBallisticVelocityDebug;
            updateYawBallisticFeedforwardDebug = false;
        }

        float ballisticFeedforward = kYawBallisticVelocity * chassisRelativeVelocityTarget;

        // float velocityFeedforward = speedTarget * GM6020_MAX_OUTPUT;  // for tuning feedforward

        chassisYawRelativeVelocityTargetDisplay = chassisRelativeVelocityTarget;
        chassisYawRelativeVelocityCurrentDisplay = modm::toDegree(RPM_TO_RADPS(gimbal->getYawMotorRPM(i)));
        chassisYawRelativeVelocityCurrentDisplay = yawVelocityFilters[0]->getValue();
        yawGimbalMotorPositionDisplay = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        yawGimbalMotorPositionTargetDisplay = gimbal->getTargetYawAxisAngle(AngleUnit::Radians);

        // for PID tunning through Ozone
        if (updateYawVelocityPIDsDebug) {
            yawVelocityPIDs[i]->pid.setP(yawVelocityPDebug);
            yawVelocityPIDs[i]->pid.setI(yawVelocityIDebug);
            yawVelocityPIDs[i]->pid.setD(yawVelocityDDebug);
            yawVelocityPIDs[i]->pid.reset();
            updateYawVelocityPIDsDebug = false;
        }

        float velocityControllerOutput = yawVelocityPIDs[i]->runController(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getYawMotorRPM(i)),
            gimbal->getYawMotorTorque(i));

        fieldRelativeYawVelocityTargetDisplay = fieldRelativeVelocityTarget;
        velocityPIDOutputDisplay = velocityControllerOutput;

        gimbal->setDesiredYawMotorOutput(i, velocityControllerOutput + ballisticFeedforward);
    }
}



float pitchAngularErrorDisplay = 0.0f;
float gravComp = 0.0f;

void GimbalFieldRelativeController::runPitchController(std::optional<float> velocityLimit) {

    kinematicPitchAngleDisplay =
        modm::toDegree(drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().getWrappedValue());

    float pitchAngularError =
        drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().minDifference(
            this->getTargetPitch(AngleUnit::Radians));

    pitchAngularErrorDisplay = pitchAngularError;

    float chassisRelativePitchTarget = gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) + pitchAngularError;

    gimbal->setTargetPitchAxisAngle(AngleUnit::Radians, chassisRelativePitchTarget);

    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        if (updatePitchPositionPIDsDebug) {
            pitchPositionCascadePIDs[i]->pid.setP(pitchPositionPDebug);
            pitchPositionCascadePIDs[i]->pid.setI(pitchPositionIDebug);
            pitchPositionCascadePIDs[i]->pid.setD(pitchPositionDDebug);
            pitchPositionCascadePIDs[i]->pid.reset();
            updatePitchPositionPIDsDebug = false;
        }
        if (forCV) {
            pitchOuterLoopCounter++;
            if (pitchOuterLoopCounter % 10 == 0) {
                fieldRelativeVelocityTarget = pitchPositionCascadePIDs[i]->runController(
                gimbal->getPitchMotorSetpointError(i, AngleUnit::Radians),
                RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)) + drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                                            src::Informants::AngularAxis::PITCH_AXIS,
                                                            AngleUnit::Radians) /
                                                            GIMBAL_PITCH_GEAR_RATIO);
        }
        } else {
            fieldRelativeVelocityTarget = pitchPositionCascadePIDs[i]->runController(
            gimbal->getPitchMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)) + drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                                            src::Informants::AngularAxis::PITCH_AXIS,
                                                            AngleUnit::Radians) /
                                                            GIMBAL_PITCH_GEAR_RATIO);
        }
            

        if (velocityLimit.has_value()) {
            fieldRelativeVelocityTarget =
                tap::algorithms::limitVal<float>(fieldRelativeVelocityTarget, -velocityLimit.value(), velocityLimit.value());
        }

        float chassisRelativeVelocityTarget =
            fieldRelativeVelocityTarget -
            (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                src::Informants::AngularAxis::PITCH_AXIS,
                AngleUnit::Radians
            ) / GIMBAL_PITCH_GEAR_RATIO);

        // Gravity compensation should be positive if the gimbal tips forward, negative if it tips backwards
        float gravityCompensationFeedforward =
            kGRAVITY *
            cosf(drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().getWrappedValue());

        float velocityFeedforward = gravityCompensationFeedforward +
                                    CHASSIS_VELOCITY_PITCH_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
                                        PITCH_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)) /*+
                                    CHASSIS_LINEAR_ACCELERATION_PITCH_COMPENSATION *
                                        drivers->kinematicInformant.getChassisLinearAccelerationInGimbalDirection()*/
            ;

        float chassisRelativeBallisticVelocityTarget = this->getTargetVelocityPitch(AngleUnit::Radians) - (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                               src::Informants::AngularAxis::PITCH_AXIS,
                                               AngleUnit::Radians) /
                                           GIMBAL_PITCH_GEAR_RATIO);
        if (updatePitchBallisticFeedforwardDebug) {
            kPitchBallisticVelocity = kPitchBallisticVelocityDebug;
            updateYawBallisticFeedforwardDebug = false;
        }

        float ballisticFeedforward = gravityCompensationFeedforward + kPitchBallisticVelocity * chassisRelativeBallisticVelocityTarget; 

        // chassisRelativeVelocityTargetDisplay = chassisRelativeVelocityTarget;
        // chassisRelativeVelocityCurrentDisplay = RPM_TO_RADPS(gimbal->getPitchMotorRPM(i));
        pitchGimbalMotorPositionDisplay = gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians);
        pitchGimbalMotorPositionTargetDisplay = gimbal->getTargetPitchAxisAngle(AngleUnit::Radians);
        pitchVelocityFeedforwardOutputDisplay = velocityFeedforward;

        if (updatePitchVelocityPIDsDebug) {
            pitchVelocityPIDs[i]->pid.setP(pitchVelocityPDebug);
            pitchVelocityPIDs[i]->pid.setI(pitchVelocityIDebug);
            pitchVelocityPIDs[i]->pid.setD(pitchVelocityDDebug);
            pitchVelocityPIDs[i]->pid.reset();
            updatePitchVelocityPIDsDebug = false;
        }

        float velocityControllerOutput = pitchVelocityPIDs[i]->runController(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)),
            gimbal->getPitchMotorTorque(i));

        pitchOutputVelocityDisplay = velocityFeedforward + velocityControllerOutput;
        if (gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) < PITCH_AXIS_SOFTSTOP_HIGH + 0.0873 &&
            gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) > PITCH_AXIS_SOFTSTOP_LOW - 0.0873) {
            if (forCV) gimbal->setDesiredPitchMotorOutput(i, velocityControllerOutput + ballisticFeedforward);
            else gimbal->setDesiredPitchMotorOutput(i, velocityFeedforward + velocityControllerOutput);
        } else {
            gimbal->setDesiredPitchMotorOutput(i, 10000 * sgn(kGRAVITY));
        }
    }
}

void GimbalFieldRelativeController::runPitchVelocityController(std::optional<float> velocityLimit) {
    kinematicPitchAngleDisplay =
        modm::toDegree(drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().getWrappedValue());

    fieldRelativePitchVelocityTargetDisplay = this->getTargetVelocityPitch(AngleUnit::Degrees);
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        pitchVelocityFilters[i]->update(modm::toDegree(RPM_TO_RADPS(gimbal->getPitchMotorRPM(i))));

        float fieldRelativeVelocityTarget = this->getTargetVelocityPitch(AngleUnit::Radians);

        if (velocityLimit.has_value()) {
            fieldRelativeVelocityTarget =
                tap::algorithms::limitVal<float>(fieldRelativeVelocityTarget, -velocityLimit.value(), velocityLimit.value());
        }

        float chassisRelativeVelocityTarget =
            fieldRelativeVelocityTarget -
            (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                src::Informants::AngularAxis::PITCH_AXIS,
                AngleUnit::Radians)
            / GIMBAL_PITCH_GEAR_RATIO);

        // Gravity compensation should be positive if the gimbal tips forward, negative if it tips backwards
        float gravityCompensationFeedforward =
            kGRAVITY *
            cosf(drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().getWrappedValue());

        float velocityFeedforward = gravityCompensationFeedforward +
                                    CHASSIS_VELOCITY_PITCH_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
                                        PITCH_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)) /*+
                                    CHASSIS_LINEAR_ACCELERATION_PITCH_COMPENSATION *
                                        drivers->kinematicInformant.getChassisLinearAccelerationInGimbalDirection()*/
            ;

        float chassisRelativeBallisticVelocityTarget = this->getTargetVelocityPitch(AngleUnit::Radians) - (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                               src::Informants::AngularAxis::PITCH_AXIS,
                                               AngleUnit::Radians) /
                                           GIMBAL_PITCH_GEAR_RATIO);

        if (updatePitchBallisticFeedforwardDebug) {
            kPitchBallisticVelocity = kPitchBallisticVelocityDebug;
            updateYawBallisticFeedforwardDebug = false;
        }

        float ballisticFeedforward = gravityCompensationFeedforward + kPitchBallisticVelocity * chassisRelativeVelocityTarget; 

        chassisPitchRelativeVelocityTargetDisplay = chassisRelativeVelocityTarget;
        chassisPitchRelativeVelocityCurrentDisplay = modm::toDegree(RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)));
        chassisPitchRelativeVelocityCurrentDisplay = pitchVelocityFilters[0]->getValue();
        pitchGimbalMotorPositionDisplay = gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians);
        pitchGimbalMotorPositionTargetDisplay = gimbal->getTargetPitchAxisAngle(AngleUnit::Radians);
        pitchVelocityFeedforwardOutputDisplay = velocityFeedforward;

        if (updatePitchVelocityPIDsDebug) {
            pitchVelocityPIDs[i]->pid.setP(pitchVelocityPDebug);
            pitchVelocityPIDs[i]->pid.setI(pitchVelocityIDebug);
            pitchVelocityPIDs[i]->pid.setD(pitchVelocityDDebug);
            pitchVelocityPIDs[i]->pid.reset();
            updatePitchVelocityPIDsDebug = false;
        }

        float velocityControllerOutput = pitchVelocityPIDs[i]->runController(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)),
            gimbal->getPitchMotorTorque(i));

        pitchOutputVelocityDisplay = velocityFeedforward + velocityControllerOutput;
        if (gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) < PITCH_AXIS_SOFTSTOP_HIGH + 0.0873 &&
            gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) > PITCH_AXIS_SOFTSTOP_LOW - 0.0873) {
            if (forCV) gimbal->setDesiredPitchMotorOutput(i, velocityControllerOutput + ballisticFeedforward);
            else gimbal->setDesiredPitchMotorOutput(i, velocityFeedforward + velocityControllerOutput);
        } else {
            gimbal->setDesiredPitchMotorOutput(i, 10000 * sgn(kGRAVITY));
        }
    }
}
bool GimbalFieldRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal

#endif  // GIMBAL_COMPATIBLE
#endif  // GIMBAL_UNTETHERED
