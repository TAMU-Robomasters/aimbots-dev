#include "gimbal_field_relative_controller.hpp"
#include "tap/motor/dji_motor_encoder.hpp"
#include "modm/math/geometry/angle.hpp"
#include "utils/tools/common_types.hpp"

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
    yawVelocityPID->pid.reset();
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        yawPositionPIDs[i]->pid.reset();
    }
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        pitchPositionPIDs[i]->pid.reset();
        pitchVelocityPIDs[i]->pid.reset();
    }
}

float fieldRelativeYawTargetDisplay = 0.0f;
float targetYawAxisAngleDisplay = 0.0f;
float fieldRelativeYawTargetRadDisplay = 0.0f;

float fieldRelativeYawOutputDisplay = 0.0f;

float feedforwardDisplay = 0.0f;

float fieldRelativeYawVelocityTargetDisplay = 0.0f;
float fieldRelativePitchVelocityTargetDisplay = 0.0f;
float fieldRelativeVelocityTargetDisplay = 0.0f;
float velocityFeedforwardOutputDisplay = 0.0f;
float pitchVelocityFeedforwardOutputDisplay = 0.0f;
float yawVelocityFeedforwardOutputDisplay = 0.0f;
float velocityPIDOutputDisplay = 0.0f;

float chassisYawRelativeVelocityTargetDisplay = 0.0f;
float chassisYawRelativeVelocityCurrentDisplay = 0.0f;
float chassisYawRelativeDerivedVelocityDisplay = 0.0f;

float chassisPitchRelativeVelocityTargetDisplay = 0.0f;
float chassisPitchRelativeVelocityCurrentDisplay = 0.0f;
float chassisPitchRelativeDerivedVelocityDisplay = 0.0f;

float chassisRelativeVelocityTargetDisplay = 0.0f;
float chassisRelativeVelocityCurrentDisplay = 0.0f;
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
float kinematicYawAngleRadDisplay = 0;

float chassisIMUAngularVelocityFromKIDisplay;

float speedTarget = 0.0f;
float speedDiffDisplay = 0.0f;

// ozone pid tuning

float gimbalYawPositionCascadePDebug = 0.0f;
float gimbalYawPositionCascadeIDebug = 0.0f;
float gimbalYawPositionCascadeDDebug = 0.0f;
bool updateGimbalYawPositionCascadeDebug = false;

float yawVelocityErrorDisplay = 0.0f;

void GimbalFieldRelativeController::runYawController(
    std::optional<float> velocityLimit) {  // using cascade controller for yaw

    fieldRelativeYawTargetRadDisplay = this->getTargetYaw(AngleUnit::Radians);
    fieldRelativeYawTargetDisplay = this->getTargetYaw(AngleUnit::Degrees);

    kinematicYawAngleRadDisplay = drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue();

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

    averageRPM = 0.0f;
    for(int i=0;i<YAW_MOTOR_COUNT;i++){
        averageRPM += gimbal->getYawMotorRPM(i);
    }
    averageRPM = averageRPM / YAW_MOTOR_COUNT;

    yawVelocityFilter->update(modm::toDegree(RPM_TO_RADPS(averageRPM)));

    if (updateGimbalYawPositionCascadeDebug) {
            yawPositionCascadePID->pid.setP(gimbalYawPositionCascadePDebug);
            yawPositionCascadePID->pid.setI(gimbalYawPositionCascadeIDebug);
            yawPositionCascadePID->pid.setD(gimbalYawPositionCascadeDDebug);
            yawPositionCascadePID->pid.reset();
            updateGimbalYawPositionCascadeDebug = false;
    }
    

    // speedTarget += 1 / 200.0f;
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {

    #ifndef YAW_3508
        float fieldRelativeVelocityTarget = yawPositionCascadePID->runController(
            gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getYawMotorRPM(i)) + drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                                          src::Informants::AngularAxis::YAW_AXIS,
                                                          AngleUnit::Radians) /
                                                          GIMBAL_YAW_GEAR_RATIO);

    #elif defined (TARGET_HERO) 
        float fieldRelativeVelocityTarget = yawPositionCascadePID->runController(
            gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getYawMotorRPM(i)) + drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                                          src::Informants::AngularAxis::YAW_AXIS,
                                                          AngleUnit::Radians) /
                                                          GIMBAL_YAW_GEAR_RATIO);
    #else //WARN: this is for sentry but for some reason it doesn't convert RPM to radians per sec
        float fieldRelativeVelocityTarget = yawPositionCascadePID->runController(
            gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getYawMotorRPM(i)) + drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                                          src::Informants::AngularAxis::YAW_AXIS,
                                                          AngleUnit::Radians) /
                                                          GIMBAL_YAW_GEAR_RATIO);
    #endif

        // float chassisRelativeVelocityTarget = sinf(speedTarget) * 2.0f;

    if (velocityLimit.has_value()) {
        fieldRelativeVelocityTarget =
            tap::algorithms::limitVal<float>(fieldRelativeVelocityTarget, -velocityLimit.value(), velocityLimit.value());
    }
    #if  defined(ALL_SENTRIES) || defined(ALL_HEROES) 
        float chassisRelativeVelocityTarget =
            fieldRelativeVelocityTarget - (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                src::Informants::AngularAxis::YAW_AXIS,
                AngleUnit::Radians)
                *GIMBAL_YAW_MOTOR_GEAR_RATIO);
                
                speedDiffDisplay = (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                src::Informants::AngularAxis::YAW_AXIS,
                AngleUnit::Radians)*GIMBAL_YAW_MOTOR_GEAR_RATIO);
    #else
        float chassisRelativeVelocityTarget =
                fieldRelativeVelocityTarget - (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                                   src::Informants::AngularAxis::YAW_AXIS,
                                                   AngleUnit::Radians) /
                                               GIMBAL_YAW_GEAR_RATIO);
    #endif


    #ifndef YAW_3508
        float velocityFeedforward = tap::algorithms::limitVal(
            CHASSIS_VELOCITY_YAW_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
                YAW_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)),
            -GM6020_MAX_OUTPUT,
            GM6020_MAX_OUTPUT);
    // #elif defined (ALL_HEROES)
    //     float chassisRelativeVelocityThreshold = 0.1;
    //     float velocityFeedforward = tap::algorithms::limitVal(
    //         (chassisRelativeVelocityTarget > chassisRelativeVelocityThreshold) * sgn(chassisRelativeVelocityTarget) * chassisRelativeYawFeedforward(fabs(chassisRelativeVelocityTarget)),
    //         -M3508_MAX_OUTPUT,
    //         M3508_MAX_OUTPUT);   
    #else
        // float velocityFeedforward = tap::algorithms::limitVal(
        //     CHASSIS_VELOCITY_YAW_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
        //         YAW_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)),
        //     -M3508_MAX_OUTPUT,
        //     M3508_MAX_OUTPUT);
        float velocityFeedforward = 0.0f;
    #endif

        // float velocityFeedforward = speedTarget * GM6020_MAX_OUTPUT;  // for tuning feedforward

        chassisRelativeVelocityTargetDisplay = chassisRelativeVelocityTarget;
       // chassisRelativeVelocityCurrentDisplay = gimbal->getYawMotorRPM(i)/GIMBAL_YAW_MOTOR_GEAR_RATIO;
         //chassisRelativeVelocityCurrentDisplay = yawVelocityFilter->getValue()/GIMBAL_YAW_MOTOR_GEAR_RATIO;
        yawGimbalMotorPositionDisplay = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        yawGimbalMotorPositionTargetDisplay = gimbal->getTargetYawAxisAngle(AngleUnit::Radians);

    #ifndef YAW_3508
        float velocityControllerOutput = yawVelocityPID->runController(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getYawMotorRPM(i)),
            gimbal->getYawMotorTorque(i));
        gimbal->setDesiredYawMotorOutput(i, /*velocityFeedforward + */velocityControllerOutput);
    #elif defined (ALL_HEROES)
        float velocityControllerOutput = yawVelocityPID->runController(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getYawMotorRPM(i)),
            gimbal->getYawMotorTorque(i));
        gimbal->setDesiredYawMotorOutput(i, velocityFeedforward + velocityControllerOutput);
    #else //WARN: for some reason sentry not using motor torque for derivative? torque proportionally to accel
        float velocityControllerOutput = yawVelocityPID->runControllerDerivateError(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getYawMotorRPM(i)));
        gimbal->setDesiredYawMotorOutput(i,velocityControllerOutput + velocityFeedforward);
    #endif
        yawVelocityErrorDisplay = chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getYawMotorRPM(i));

        fieldRelativeVelocityTargetDisplay = fieldRelativeVelocityTarget;
        velocityPIDOutputDisplay = velocityControllerOutput;

       // gimbal->setDesiredOutputToYawMotor(i);
    }
}

float pitchAngularErrorDisplay = 0.0f;
float gravComp = 0.0f;
float overshootPitchMotorDisplay = 0.0f;
float pitchAngleDisplay = 0.0f;
float softstopHighDisplay = 0.0f;
float softstopLowDisplay = 0.0f;

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

float kinematicPitchAngleDisplay = 0.0f;
float velocityErrorDisplay = 69.0f;
float chassisVelocityDisplay = 0.0f;

// for PID testing
void GimbalFieldRelativeController::runYawVelocityController(
    std::optional<float> velocityLimit) {
    
    kinematicYawAngleDisplay =
        modm::toDegree(drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue());


    averageRPM = 0.0f;
    for(int i=0;i<YAW_MOTOR_COUNT;i++){
        averageRPM += gimbal->getYawMotorRPM(i);
    }
    averageRPM = averageRPM / YAW_MOTOR_COUNT;

    //TODO: get rid of me when you retunned sentry
    #if  defined(ALL_SENTRIES) || defined(ALL_HEROES) 
        yawVelocityFilter->update(modm::toDegree(RPM_TO_RADPS(averageRPM)));
    #else
        yawVelocityFilter->update(modm::toDegree(RPM_TO_RADPS(averageRPM)));
    #endif

        // for PID tunning through Ozone
    if (updateYawVelocityPIDsDebug) {
        yawVelocityPID->pid.setP(yawVelocityPDebug);
        yawVelocityPID->pid.setI(yawVelocityIDebug);
        yawVelocityPID->pid.setD(yawVelocityDDebug);
        yawVelocityPID->pid.reset();
        updateYawVelocityPIDsDebug = false;
    }

    // speedTarget += 1 / 200.0f;
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        float fieldRelativeVelocityTarget = this->getTargetVelocityYaw(AngleUnit::Radians);

        if (velocityLimit.has_value()) {
            fieldRelativeVelocityTarget =
                tap::algorithms::limitVal<float>(fieldRelativeVelocityTarget, -velocityLimit.value(), velocityLimit.value());
        }
        fieldRelativeYawVelocityTargetDisplay = modm::toDegree(fieldRelativeVelocityTarget);

        #if  defined(ALL_SENTRIES) || defined(ALL_HEROES)  //TODO: fix this after tunning sentry
            float chassisRelativeVelocityTarget =
                fieldRelativeVelocityTarget - (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                                   src::Informants::AngularAxis::YAW_AXIS,
                                                   AngleUnit::Radians) * GIMBAL_YAW_MOTOR_GEAR_RATIO);
        #else 
            float chassisRelativeVelocityTarget =
                fieldRelativeVelocityTarget - (drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                                   src::Informants::AngularAxis::YAW_AXIS,
                                                   AngleUnit::Radians) /
                                               GIMBAL_YAW_GEAR_RATIO);
        #endif


        chassisVelocityDisplay = chassisRelativeVelocityTarget;


        
        #if  defined(ALL_SENTRIES) || defined(ALL_HEROES) 
          float velocityFeedforward = tap::algorithms::limitVal(
              0.135f * sgn(chassisRelativeVelocityTarget) * chassisRelativeVelocityYawFeedforward(fabs(chassisRelativeVelocityTarget)),
              -M3508_MAX_OUTPUT,
              M3508_MAX_OUTPUT);
        // #elif  defined (ALL_HEROES)
        //     float velocityFeedforward = tap::algorithms::limitVal(
        //         sgn(chassisRelativeVelocityTarget) * chassisRelativeYawFeedforward(chassisRelativeVelocityTarget),
        //         -M3508_MAX_OUTPUT,
        //         M3508_MAX_OUTPUT);
        #else
        float velocityFeedforward = 0;
        #endif

        chassisYawRelativeVelocityTargetDisplay = chassisRelativeVelocityTarget;
        chassisYawRelativeVelocityCurrentDisplay = yawVelocityFilter->getValue();
        //chassisYawRelativeVelocityCurrentDisplay = yawVelocityFilters[0]->getValue();
        yawGimbalMotorPositionDisplay = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        yawGimbalMotorPositionTargetDisplay = gimbal->getTargetYawAxisAngle(AngleUnit::Radians);

    #ifndef YAW_3508
        float velocityControllerOutput = yawVelocityPID->runController(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getYawMotorRPM(i)),
            gimbal->getYawMotorTorque;
    // #elif defined (ALL_HEROES)
    //      float velocityControllerOutput = yawVelocityPID->runController(
    //         chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getYawMotorRPM(i)),
    //         gimbal->getYawMotorTorque(i));
    //     gimbal->setDesiredYawMotorOutput(i, velocityFeedforward + velocityControllerOutput);
    #else //WARN: for some reason sentry not using motor torque for derivative? torque proportionally to accel
        float velocityControllerOutput = yawVelocityPID->runControllerDerivateError(
            chassisRelativeVelocityTarget - RPM_TO_RADPS(gimbal->getYawMotorRPM(i)));
    #endif
        velocityErrorDisplay = chassisRelativeVelocityTarget;
        velocityPIDOutputDisplay = velocityControllerOutput;
        feedforwardDisplay = velocityFeedforward;

        gimbal->setDesiredYawMotorOutput(i, velocityControllerOutput + velocityFeedforward);
    }
}


void GimbalFieldRelativeController::runPitchController(std::optional<float> velocityLimit) {
    float pitchAngularError = 
        drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().minDifference(
            this->getTargetPitch(AngleUnit::Radians));

    pitchAngularErrorDisplay = pitchAngularError;

    float chassisRelativePitchTarget = gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) + pitchAngularError;
    pitchAngleDisplay = gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians);
    gimbal->setTargetPitchAxisAngle(AngleUnit::Radians, chassisRelativePitchTarget);


         // for PID tunning through Ozone
    

    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) { 
        if (updatePitchPositionPIDsDebug) {
            pitchPositionCascadePIDs[i]->pid.setP(pitchPositionPDebug);
            pitchPositionCascadePIDs[i]->pid.setI(pitchPositionIDebug);
            pitchPositionCascadePIDs[i]->pid.setD(pitchPositionDDebug);
            pitchPositionCascadePIDs[i]->pid.reset();
            updatePitchPositionPIDsDebug = false;
        }
        float fieldRelativeVelocityTarget = pitchPositionCascadePIDs[i]->runController(
            gimbal->getPitchMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)) + drivers->kinematicInformant.getChassisIMUAngularVelocity(
                                                            src::Informants::AngularAxis::PITCH_AXIS,
                                                            AngleUnit::Radians) /
                                                            GIMBAL_PITCH_GEAR_RATIO);

        if (velocityLimit.has_value()) {
            fieldRelativeVelocityTarget =
                tap::algorithms::limitVal<float>(fieldRelativeVelocityTarget, -velocityLimit.value(), velocityLimit.value());
        }

        float chassisRelativeVelocityTarget =
            fieldRelativeVelocityTarget -
            (drivers->kinematicInformant.getChassisPitchVelocityInGimbalDirection() / GIMBAL_PITCH_GEAR_RATIO);

        // Gravity compensation should be positive if the gimbal tips forward, negative if it tips backwards
        float gravityCompensationFeedforward =
            kGRAVITY *
            cosf(drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().getWrappedValue());

        float velocityFeedforward = gravityCompensationFeedforward +
                                    CHASSIS_VELOCITY_PITCH_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
                                        PITCH_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)) /*+
                                    CHASSIS_LINEAR_ACCELERATION_PITCH_COMPENSATION *
                                        drivers->kinematicInformant.getChassisLinearAccelerationInGimbalDirection()*/
                                #ifdef TARGET_SENTRY
                                    + fieldRelativePitchAngleFeedforward.interpolate(this->getTargetVelocityPitch(AngleUnit::Radians))
                                #endif

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
        softstopHighDisplay = PITCH_AXIS_SOFTSTOP_HIGH + 0.0873;
        softstopLowDisplay = PITCH_AXIS_SOFTSTOP_LOW - 0.0873;
        if (gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) < PITCH_AXIS_SOFTSTOP_HIGH + 0.0873 &&
            gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) > PITCH_AXIS_SOFTSTOP_LOW - 0.0873) {
            gimbal->setDesiredPitchMotorOutput(i, velocityFeedforward + velocityControllerOutput);
            overshootPitchMotorDisplay = 67.0f;
        } else if(gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) > PITCH_AXIS_SOFTSTOP_HIGH + 0.0873) { // baconsizzle notes
            gimbal->setDesiredPitchMotorOutput(i, -10000 /** sgn(kGRAVITY)*/);
            overshootPitchMotorDisplay = -10000 /** sgn(kGRAVITY)*/;
        }else if(gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) < PITCH_AXIS_SOFTSTOP_LOW - 0.0873){
            gimbal->setDesiredPitchMotorOutput(i, 10000 /** sgn(kGRAVITY)*/);
            overshootPitchMotorDisplay = 10000 /** sgn(kGRAVITY)*/;
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

        float velocityFeedforward = gravityCompensationFeedforward /*+
                                    CHASSIS_VELOCITY_PITCH_LOAD_FEEDFORWARD * sgn(chassisRelativeVelocityTarget) *
                                        PITCH_VELOCITY_FEEDFORWARD.interpolate(fabs(chassisRelativeVelocityTarget)) *//*+
                                    CHASSIS_LINEAR_ACCELERATION_PITCH_COMPENSATION *
                                        drivers->kinematicInformant.getChassisLinearAccelerationInGimbalDirection()*/
                                #ifdef TARGET_SENTRY
                                    + fieldRelativePitchAngleFeedforward.interpolate(this->getTargetVelocityPitch(AngleUnit::Radians))
                                #endif
            ;

        chassisPitchRelativeVelocityTargetDisplay = chassisRelativeVelocityTarget;
       // chassisPitchRelativeVelocityCurrentDisplay = modm::toDegree(RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)));
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

        float totalOutput =velocityFeedforward + velocityControllerOutput;
        if(totalOutput > GM6020_MAX_OUTPUT){
            totalOutput = GM6020_MAX_OUTPUT;
        }else if(totalOutput < -GM6020_MAX_OUTPUT){
            totalOutput = -GM6020_MAX_OUTPUT;
        }

        pitchOutputVelocityDisplay = totalOutput;
        // if (gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) < PITCH_AXIS_SOFTSTOP_HIGH &&
        //     gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians) > PITCH_AXIS_SOFTSTOP_LOW ) {
            gimbal->setDesiredPitchMotorOutput(i, totalOutput);
    //     } else {
    //         gimbal->setDesiredPitchMotorOutput(i, -3000 * sgn(kGRAVITY));
    //     }
     }
}

bool GimbalFieldRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal

#endif  // GIMBAL_COMPATIBLE
#endif  // GIMBAL_UNTETHERED
