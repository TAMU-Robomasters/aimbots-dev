#include "kinematic_informant.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/tools/robot_specific_inc.hpp"

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

Vector3f KinematicInformant::getLocalIMUAngles() {
    Vector3f imuAngles = {
        -drivers->bmi088.getPitch(),  // inverts pitch
        drivers->bmi088.getRoll(),
        drivers->bmi088.getYaw() - 180.0f};  // for some reason yaw is 180.0 degrees rotated
    return imuAngles * (M_PI / 180.0f);      // Convert to rad
}
float KinematicInformant::getLocalIMUAngle(AngularAxis axis) {  // Gets IMU angles in IMU Frame
    switch (axis) {
        case PITCH_AXIS:
            return -modm::toRadian(drivers->bmi088.getPitch());
        case ROLL_AXIS:
            return modm::toRadian(drivers->bmi088.getRoll());
        case YAW_AXIS:
            return modm::toRadian(drivers->bmi088.getYaw() - 180.0f);  // for some reason yaw is 180.0 degrees rotated
    }
    return 0;
}

Vector3f KinematicInformant::getIMUAngularVelocities() {  // Gets IMU Angular Velocity in IMU FRAME
    Vector3f imuAngularVelocities = {-drivers->bmi088.getGy(), drivers->bmi088.getGx(), drivers->bmi088.getGz()};
    return imuAngularVelocities * (M_PI / 180.0f);  // Convert to rad/s
}

float KinematicInformant::getIMUAngularVelocity(AngularAxis axis) {  // Gets IMU angles in IMU Frame
    switch (axis) {
        case PITCH_AXIS:
            return -modm::toRadian(drivers->bmi088.getGy());
        case ROLL_AXIS:
            return modm::toRadian(drivers->bmi088.getGx());
        case YAW_AXIS:
            return modm::toRadian(drivers->bmi088.getGz());
    }
    return 0;
}

// Update IMU Kinematic State Vectors
// All relative to IMU Frame
void KinematicInformant::updateIMUKinematicStateVector() {
    imuLinearState[X_AXIS].updateFromAcceleration(-drivers->bmi088.getAy());
    imuLinearState[Y_AXIS].updateFromAcceleration(drivers->bmi088.getAx());
    imuLinearState[Z_AXIS].updateFromAcceleration(drivers->bmi088.getAz());

    imuAngularState[X_AXIS].updateFromPosition(getLocalIMUAngle(PITCH_AXIS));
    imuAngularState[Y_AXIS].updateFromPosition(getLocalIMUAngle(ROLL_AXIS));
    imuAngularState[Z_AXIS].updateFromPosition(getLocalIMUAngle(YAW_AXIS));

    imuAngularState[X_AXIS].updateFromVelocity(getIMUAngularVelocity(PITCH_AXIS), false);
    imuAngularState[Y_AXIS].updateFromVelocity(getIMUAngularVelocity(ROLL_AXIS), false);
    imuAngularState[Z_AXIS].updateFromVelocity(getIMUAngularVelocity(YAW_AXIS), false);
}

Vector3f KinematicInformant::getIMUAngularAccelerations() {
    float alphax = imuAngularState[X_AXIS].getAcceleration();
    float alphay = imuAngularState[Y_AXIS].getAcceleration();
    float alphaz = imuAngularState[Z_AXIS].getAcceleration();

    Vector3f alpha = {alphax, alphay, alphaz};
    return alpha;
}

Vector3f KinematicInformant::getIMULinearAccelerations() {
    float ax = -drivers->bmi088.getAy();
    float ay = drivers->bmi088.getAx();
    float az = drivers->bmi088.getAz();

    Vector3f a = {ax, ay, az};
    return a;
}

float KinematicInformant::getIMULinearAcceleration(LinearAxis axis) {  // Gets IMU accel in IMU Frame
    switch (axis) {
        case X_AXIS:
            return drivers->bmi088.getAx();
        case Y_AXIS:
            return drivers->bmi088.getAy();
        case Z_AXIS:
            return drivers->bmi088.getAz();
    }
    return 0;
}

Vector3f chassisAnglesConvertedDisplay;
Vector3f IMUAnglesDisplay;
void KinematicInformant::updateChassisIMUAngles() {
    Vector3f IMUAngles = getLocalIMUAngles();
    Vector3f IMUAngularVelocities = getIMUAngularVelocities();

    IMUAnglesDisplay = IMUAngles;

    // Gets chassis angles
    Vector3f chassisAngles =
        drivers->kinematicInformant.getRobotFrames()
            .getFrame(Transformers::FrameType::CHASSIS_IMU_FRAME)
            .getPointInFrame(
                drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::CHASSIS_FRAME),
                IMUAngles);

    chassisAnglesConvertedDisplay = chassisAngles;

    Vector3f chassisAngularVelocities =
        drivers->kinematicInformant.getRobotFrames()
            .getFrame(Transformers::FrameType::CHASSIS_IMU_FRAME)
            .getPointInFrame(
                drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::CHASSIS_FRAME),
                IMUAngularVelocities);

    chassisAngularState[X_AXIS].updateFromPosition(chassisAngles[X_AXIS]);
    chassisAngularState[Y_AXIS].updateFromPosition(chassisAngles[Y_AXIS]);
    chassisAngularState[Z_AXIS].updateFromPosition(chassisAngles[Z_AXIS]);

    chassisAngularState[X_AXIS].updateFromVelocity(chassisAngularVelocities[X_AXIS], false);
    chassisAngularState[Y_AXIS].updateFromVelocity(chassisAngularVelocities[Y_AXIS], false);
    chassisAngularState[Z_AXIS].updateFromVelocity(chassisAngularVelocities[Z_AXIS], false);
}

float KinematicInformant::getChassisIMUAngle(AngularAxis axis, AngleUnit unit) {
    float angle = chassisAngularState[axis].getPosition();

    return unit == AngleUnit::Radians ? angle : modm::toDegree(angle);
}

float KinematicInformant::getChassisIMUAngularVelocity(AngularAxis axis, AngleUnit unit) {
    float angularVelocity = chassisAngularState[axis].getVelocity();

    return unit == AngleUnit::Radians ? angularVelocity : modm::toDegree(angularVelocity);
}

float KinematicInformant::getIMUAngularAcceleration(AngularAxis axis, AngleUnit unit) {
    float angularAcceleration = imuAngularState[axis].getAcceleration();

    return unit == AngleUnit::Radians ? angularAcceleration : modm::toDegree(angularAcceleration);
}

Vector3f wDisplay = {0.0f, 0.0f, 0.0f};
Vector3f alphaDisplay = {0.0f, 0.0f, 0.0f};
Vector3f rDisplay = {0.0f, 0.0f, 0.0f};
Vector3f aDisplay = {0.0f, 0.0f, 0.0f};

float aXDisplay = 0.0f;
float aYDisplay = 0.0f;
float aZDisplay = 0.0f;
Vector3f KinematicInformant::removeFalseAcceleration(
    Vector<KinematicStateVector, 3> imuLinearKSV,
    Vector<KinematicStateVector, 3> imuAngularKSV,
    Vector3f r) {
    Vector3f w = {
        imuAngularKSV[X_AXIS].getVelocity(),
        imuAngularKSV[Y_AXIS].getVelocity(),
        imuAngularKSV[Z_AXIS].getVelocity()};

    // Vector3f alpha = {
    //     imuAngularKSV[X_AXIS].getAcceleration(),
    //     imuAngularKSV[Y_AXIS].getAcceleration(),
    //     imuAngularKSV[Z_AXIS].getAcceleration()};
    // was broken so i made it 0, requires investigation on why it's broken (was setting stuff to infinity)

    Vector3f alpha = {0.0f, 0.0f, 0.0f};

    Vector3f a = {
        imuLinearKSV[X_AXIS].getAcceleration(),
        imuLinearKSV[Y_AXIS].getAcceleration(),
        imuLinearKSV[Z_AXIS].getAcceleration()};

    aXDisplay = a.getX();
    aYDisplay = a.getY();
    aZDisplay = a.getZ();

    wDisplay = w;
    alphaDisplay = alpha;
    rDisplay = r;
    aDisplay = a;

    Vector3f linearIMUAcceleration = a - (alpha ^ r) - (w ^ (w ^ r));
    return linearIMUAcceleration;
}

Vector3f linearIMUAccelerationDisplay;
float linearIMUAccelerationXDisplay = 0.0f;
float linearIMUAccelerationYDisplay = 0.0f;
float linearIMUAccelerationZDisplay = 0.0f;

float chassisAngleXDisplay = 0.0f;
float chassisAngleYDisplay = 0.0f;
float chassisAngleZDisplay = 0.0f;
void KinematicInformant::updateChassisAcceleration() {
    chassisAngleXDisplay = chassisAngularState[X_AXIS].getPosition();
    chassisAngleYDisplay = chassisAngularState[Y_AXIS].getPosition();
    chassisAngleZDisplay = chassisAngularState[Z_AXIS].getPosition();

    Vector3f linearIMUAcceleration = removeFalseAcceleration(imuLinearState, imuAngularState, IMU_MOUNT_POSITION);

    Vector3f linearChassisAcceleration =
        drivers->kinematicInformant.getRobotFrames()
            .getFrame(Transformers::FrameType::CHASSIS_IMU_FRAME)
            .getPointInFrame(
                drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::CHASSIS_FRAME),
                linearIMUAcceleration);

    linearIMUAccelerationDisplay = linearIMUAcceleration;
    linearIMUAccelerationXDisplay = linearChassisAcceleration.getX();
    linearIMUAccelerationYDisplay = linearChassisAcceleration.getY();
    linearIMUAccelerationZDisplay = linearChassisAcceleration.getZ();

    chassisLinearState[X_AXIS].updateFromAcceleration(linearChassisAcceleration.getX());
    chassisLinearState[Y_AXIS].updateFromAcceleration(linearChassisAcceleration.getY());
    chassisLinearState[Z_AXIS].updateFromAcceleration(linearChassisAcceleration.getZ());
}

float chassisYawAngleDisplay = 0.0f;
tap::algorithms::WrappedFloat KinematicInformant::getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat() {
    float currGimbalAngle = gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians);
    float currChassisAngle = getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians);
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

void KinematicInformant::updateRobotFrames() {
    // Update IMU Stuff
    updateIMUKinematicStateVector();

#ifndef TARGET_TURRET
    // Update Chassis Stuff after IMU STUFF
    updateChassisIMUAngles();
    updateChassisAcceleration();

    chassisIMUHistoryBuffer.prependOverwrite(
        {getChassisIMUAngle(PITCH_AXIS, AngleUnit::Radians),
         getChassisIMUAngle(ROLL_AXIS, AngleUnit::Radians),
         getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians)});

    chassisKFOdometry.update(
        getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians),
        chassisLinearState[X_AXIS].getAcceleration(),
        chassisLinearState[Y_AXIS].getAcceleration());

    // update gimbal orientation buffer
    std::pair<float, float> orientation;
    orientation.first = getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue();
    orientation.second = getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().getWrappedValue();

    gimbalFieldOrientationBuffer.prependOverwrite(orientation);

    modm::Location2D<float> robotLocation = chassisKFOdometry.getCurrentLocation2D();

    robotLocationXDisplay = robotLocation.getX();
    robotLocationYDisplay = robotLocation.getY();

    robotFrames.updateFrames(
        gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians),
        gimbalSubsystem->getCurrentPitchAxisAngle(AngleUnit::Radians),
        getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians) + CHASSIS_START_ANGLE_WORLD,
        {robotLocation.getX(), robotLocation.getY(), 0},
        AngleUnit::Radians);

    turretFrames.updateFrames(
        getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians),
        getChassisIMUAngle(PITCH_AXIS, AngleUnit::Radians),
        getChassisIMUAngle(ROLL_AXIS, AngleUnit::Radians),
        AngleUnit::Radians);

    robotLocationDisplay = robotLocation;
#endif
}

float frameDelayDisplay = 0;

void KinematicInformant::mirrorPastRobotFrame(uint32_t frameDelay_ms) {
    frameDelayDisplay = frameDelay_ms;

    std::pair<float, float> gimbalAngles = gimbalSubsystem->getGimbalOrientationAtTime(frameDelay_ms);

    robotFrames.mirrorPastCameraFrame(gimbalAngles.first, gimbalAngles.second, AngleUnit::Radians);

    std::pair<float, float> gimbalFieldAngles = getGimbalFieldOrientationAtTime(frameDelay_ms);

    turretFrames.mirrorPastCameraFrame(gimbalFieldAngles.first, gimbalFieldAngles.second, AngleUnit::Radians);
}

}  // namespace src::Informants