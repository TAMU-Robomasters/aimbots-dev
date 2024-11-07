#include "kinematic_informant.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"


#include "drivers.hpp"

namespace src::Informants {

KinematicInformant::KinematicInformant(src::Drivers* drivers)
    : drivers(drivers)
#ifndef TARGET_TURRET
      ,
      // chassisKFOdometry(-2.830f, -0.730f)
      chassisKFOdometry(3.1f, 3.5f)
#warning "don't hardcode these values"
#endif
{
}

void KinematicInformant::initialize(float imuFrequency, float imukP, float imukI) {
    drivers->bmi088.initialize(imuFrequency, imukP, imukI);
}

void KinematicInformant::recalibrateIMU(Vector3f imuCalibrationEuler) {
    // drivers->bmi088.requestRecalibration(imuCalibrationEuler);
    UNUSED(imuCalibrationEuler);
    drivers->bmi088.requestRecalibration();
};

tap::communication::sensors::imu::ImuInterface::ImuState KinematicInformant::getIMUState() {
    return drivers->bmi088.getImuState();
}




Vector3f wDisplay = {0.0f, 0.0f, 0.0f};
Vector3f alphaDisplay = {0.0f, 0.0f, 0.0f};
Vector3f rDisplay = {0.0f, 0.0f, 0.0f};
Vector3f aDisplay = {0.0f, 0.0f, 0.0f};

float aXDisplay = 0.0f;
float aYDisplay = 0.0f;
float aZDisplay = 0.0f;


Vector3f linearIMUAccelerationDisplay;
float linearIMUAccelerationXDisplay = 0.0f;
float linearIMUAccelerationYDisplay = 0.0f;
float linearIMUAccelerationZDisplay = 0.0f;

float chassisAngleXDisplay = 0.0f;
float chassisAngleYDisplay = 0.0f;
float chassisAngleZDisplay = 0.0f;

float chassisYawAngleDisplay = 0.0f;
tap::algorithms::WrappedFloat KinematicInformant::getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat() {
    float currGimbalAngle = gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians);
    float currChassisAngle = getIMUAngle(YAW_AXIS, AngleUnit::Radians);
    chassisYawAngleDisplay = currChassisAngle;
    return WrappedFloat(currGimbalAngle + currChassisAngle - YAW_AXIS_START_ANGLE, -M_PI, M_PI);
}

tap::algorithms::WrappedFloat KinematicInformant::getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat() {
    float currGimbalAngle = gimbalSubsystem->getCurrentPitchAxisAngle(AngleUnit::Radians);
    float currChassisAngle = getChassisPitchAngleInGimbalDirection();
    return WrappedFloat(currGimbalAngle + currChassisAngle - PITCH_AXIS_START_ANGLE, -M_PI, M_PI);
}

float KinematicInformant::getChassisPitchAngleInGimbalDirection() {
    float sinGimbYaw = sinf(gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians));
    float cosGimbYaw = cosf(gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians));

    float chassisRoll = getChassisIMUAngle(src::Informants::AngularAxis::ROLL_AXIS, AngleUnit::Radians);
    float sinChasRoll = sinf(chassisRoll);
    float cosChasRoll = cosf(chassisRoll);

    float chassisPitch = getChassisIMUAngle(src::Informants::AngularAxis::PITCH_AXIS, AngleUnit::Radians);
    float sinChasPitch = sinf(chassisPitch);
    float cosChasPitch = cosf(chassisPitch);

    float chassisPitchAngleInGimbalDirection = atan2f(
        cosGimbYaw * sinChasPitch + sinGimbYaw * sinChasRoll,
        sqrtf(pow2(sinGimbYaw) * pow2(cosChasRoll) + pow2(cosGimbYaw) * pow2(cosChasPitch)));

    return chassisPitchAngleInGimbalDirection;
}

float KinematicInformant::getChassisPitchVelocityInGimbalDirection() {
    float sinGimbYaw = sinf(gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians));
    float cosGimbYaw = cosf(gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians));

    float chassisRollVelocity = getChassisIMUAngularVelocity(src::Informants::AngularAxis::ROLL_AXIS, AngleUnit::Radians);
    float sinChasRollVelocity = sinf(chassisRollVelocity);
    float cosChasRollVelocity = cosf(chassisRollVelocity);

    float chassisPitchVelocity = getChassisIMUAngularVelocity(src::Informants::AngularAxis::PITCH_AXIS, AngleUnit::Radians);
    float sinChasPitchVelocity = sinf(chassisPitchVelocity);
    float cosChasPitchVelocity = cosf(chassisPitchVelocity);

    float chassisPitchVelocityInGimbalDirection = atan2f(
        cosGimbYaw * sinChasPitchVelocity + sinGimbYaw * sinChasRollVelocity,
        sqrtf(pow2(sinGimbYaw) * pow2(cosChasRollVelocity) + pow2(cosGimbYaw) * pow2(cosChasPitchVelocity)));

    return chassisPitchVelocityInGimbalDirection;
}

float chassisLinearStateXDisplay = 0.0f;
float KinematicInformant::getChassisLinearAccelerationInGimbalDirection() {
    // remember to convert linear accel from in/s^2 to m/s^2
    // @luke help pwease 🥺👉👈
    float ang = gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians);

    chassisLinearStateXDisplay = chassisLinearState[X_AXIS].getAcceleration();

    float accel =
        chassisLinearState[X_AXIS].getAcceleration() * sinf(ang) + chassisLinearState[Y_AXIS].getAcceleration() * cosf(ang);

    return accel;
}

modm::Location2D<float> robotLocationDisplay;
float robotLocationXDisplay = 0.0f;
float robotLocationYDisplay = 0.0f;



float frameDelayDisplay = 0;

void KinematicInformant::mirrorPastRobotFrame(uint32_t frameDelay_ms) {
    frameDelayDisplay = frameDelay_ms;

    std::pair<float, float> gimbalAngles = gimbalSubsystem->getGimbalOrientationAtTime(frameDelay_ms);

    robotFrames.mirrorPastCameraFrame(gimbalAngles.first, gimbalAngles.second, AngleUnit::Radians);

    std::pair<float, float> gimbalFieldAngles = getGimbalFieldOrientationAtTime(frameDelay_ms);

    turretFrames.mirrorPastCameraFrame(gimbalFieldAngles.first, gimbalFieldAngles.second, AngleUnit::Radians);
}

}  // namespace src::Informants