#include "kinematic_informant.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Informants {

KinematicInformant::KinematicInformant(src::Drivers* drivers)
    : drivers(drivers),
      chassisKFOdometry(CHASSIS_START_POSITION_RELATIVE_TO_WORLD.getX(), CHASSIS_START_POSITION_RELATIVE_TO_WORLD.getY()) {}

void KinematicInformant::initialize(float imuFrequency, float imukP, float imukI) {
    drivers->bmi088.initialize(imuFrequency, imukP, imukI);
}

void KinematicInformant::recalibrateIMU() { drivers->bmi088.requestRecalibration(); };

tap::communication::sensors::imu::ImuInterface::ImuState KinematicInformant::getIMUState() {
    return drivers->bmi088.getImuState();
}

Vector3f KinematicInformant::getIMUAngles() {  // Gets IMU angles in IMU Frame
    Vector3f imuAngles = {
        -drivers->bmi088.getPitch(),  // inverts pitch
        drivers->bmi088.getRoll(),
        drivers->bmi088.getYaw() - 180.0f};  // for some reason yaw is 180.0 degrees rotated
    return imuAngles * (M_PI / 180.0f);      // Convert to rad
}
float KinematicInformant::getIMUAngle(AngularAxis axis) {  // Gets IMU angles in IMU Frame

    switch (axis) {
        case PITCH_AXIS:
            return -drivers->bmi088.getPitch() * (M_PI / 180.0f);
        case ROLL_AXIS:
            return drivers->bmi088.getRoll() * (M_PI / 180.0f);
        case YAW_AXIS:
            return (drivers->bmi088.getYaw() - 180.0f) * (M_PI / 180.0f);
            // for some reason yaw is 180.0 degrees rotated
    }
}

Vector3f KinematicInformant::getIMUAngularVelocities() {  // Gets IMU Angular Velocity in IMU FRAME
    Vector3f imuAngularVelocities = {-drivers->bmi088.getGy(), drivers->bmi088.getGx(), drivers->bmi088.getGz()};
    return imuAngularVelocities * (M_PI / 180.0f);  // Convert to rad/s
}

float KinematicInformant::getIMUAngularVelocity(AngularAxis axis) {  // Gets IMU angles in IMU Frame

    switch (axis) {
        case PITCH_AXIS:
            return -drivers->bmi088.getGy() * (M_PI / 180.0f);
        case ROLL_AXIS:
            return drivers->bmi088.getGx() * (M_PI / 180.0f);
        case YAW_AXIS:
            return drivers->bmi088.getGz() * (M_PI / 180.0f);
    }
}

// Update IMU Kinematic State Vectors
// All relative to IMU Frame
void updateIMUKinematicStateVector() {
    imuLinearState[X_AXIS].updateFromAcceleration(drivers->bmi088.getAx());
    imuLinearState[Y_AXIS].updateFromAcceleration(drivers->bmi088.getAz());
    imuLinearState[Z_AXIS].updateFromAcceleration(drivers->bmi088.getAy());

    imuAngularState[X_AXIS].updateFromPosition(getIMUAngle(PITCH_AXIS));
    imuAngularState[Y_AXIS].updateFromPosition(getIMUAngle(ROLL_AXIS));
    imuAngularState[Z_AXIS].updateFromPosition(getIMUAngle(YAW_AXIS));

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
    float ax = drivers->bmi088.getAx();
    float ay = drivers->bmi088.getAy();
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
}

void KinematicInformant::updateChassisIMUAngles() {
    Vector3f IMUAngles = getIMUAngles();
    Vector3f IMUAngularVelocities = getIMUAngularVelocities;

    // Gets chassis angles
    Vector3f chassisAngles =
        drivers->kinematicInformant.getRobotFrames()
            .getFrame(Transformers::FrameType::CHASSIS_IMU_FRAME)
            .getPointInFrame(
                drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::CHASSIS_FRAME),
                IMUAngles);

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

    return unit == AngleUnit::Radians ? angle : toDegree(angle);
}

float KinematicInformant::getChassisIMUAngularVelocity(AngularAxis axis, AngleUnit unit) {
    float angularVelocity = 0.0f;
    switch (axis) {
        case YAW_AXIS: {
            angularVelocity = chassisAngularState[Z_AXIS].getVelocity();
            break;
        }
        case PITCH_AXIS: {
            angularVelocity = chassisAngularState[X_AXIS].getVelocity();
            break;
        }
        case ROLL_AXIS: {
            angularVelocity = chassisAngularState[Y_AXIS].getVelocity();
            break;
        }
    }
    return unit == AngleUnit::Radians ? angularVelocity : modm::toDegree(angularVelocity);
}

float KinematicInformant::getIMUAngularAcceleration(AngularAxis axis, AngleUnit unit) {
    float angularAcceleration = imuAngularState[axis].getAcceleration();

    return unit == AngleUnit::Radians ? angularAcceleration : modm::toDegree(angularAcceleration);
}

Vector3f KinematicInformant::removeFalseAcceleration(Vector3f imuLinearKSV, Vector3f imuAngularKSV, Vector3f r) {
    Vector3f w = {
        imuAngularKSV[X_AXIS].getVelocity(),
        imuAngularKSV[Y_AXIS].getVelocity(),
        imuAngularKSV[Z_AXIS].getVelocity()};

    Vector3f alpha = {
        imuAngularKSV[X_AXIS].getAcceleration(),
        imuAngularKSV[Y_AXIS].getAcceleration(),
        imuAngularKSV[Z_AXIS].getAcceleration()};

    Vector3f a = {
        imuLinearKSV[X_AXIS].getAcceleration(),
        imuLinearKSV[Y_AXIS].getAcceleration(),
        imuLinearKSV[Z_AXIS].getAcceleration()};

    Vector3f linearIMUAcceleration = a - (alpha ^ r) - (w ^ (w ^ r));
    return linearIMUAcceleration;
}

void KinematicInformant::updateChassisAcceleration() {
    Vector3f linearIMUAcceleration = removeFalseAcceleration(imuLinearState, imuAngularState, IMU_MOUNT_POSITION);

    Vector3f linearChassisAcceleration =
        drivers->kinematicInformant.getRobotFrames()
            .getFrame(Transformers::FrameType::CHASSIS_IMU_FRAME)
            .getPointInFrame(
                drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::CHASSIS_FRAME),
                linearIMUAcceleration);

    chassisLinearState[X_AXIS].updateFromAcceleration(linearChassisAcceleration.getX());
    chassisLinearState[Y_AXIS].updateFromAcceleration(linearChassisAcceleration.getY());
    chassisLinearState[Z_AXIS].updateFromAcceleration(linearChassisAcceleration.getZ());
}

ContiguousFloat KinematicInformant::getCurrentFieldRelativeGimbalYawAngleAsContiguousFloat() {
    float currGimbalAngle = gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians);
    float currChassisAngle = getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians);
    return ContiguousFloat(currGimbalAngle + currChassisAngle - YAW_AXIS_START_ANGLE, -M_PI, M_PI);
}

float pitchGimbalDirectionDisplay = 0.0f;
ContiguousFloat KinematicInformant::getCurrentFieldRelativeGimbalPitchAngleAsContiguousFloat() {
    float currGimbalAngle = gimbalSubsystem->getCurrentPitchAxisAngle(AngleUnit::Radians);
    float currChassisAngle = getChassisPitchAngleInGimbalDirection();
    pitchGimbalDirectionDisplay = currChassisAngle;
    return ContiguousFloat(currGimbalAngle + currChassisAngle - PITCH_AXIS_START_ANGLE, -M_PI, M_PI);
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

float KinematicInformant::getChassisLinearAccelerationInGimbalDirection() {
    // remember to convert linear accel from in/s^2 to m/s^2
    // @luke help pwease 🥺👉👈
    float ang = gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians);

    float accel =
        chassisLinearState[X_AXIS].getAcceleration() * sinf(ang) + chassisLinearState[Y_AXIS].getAcceleration() * cosf(ang);

    return accel;
}

modm::Location2D<float> robotLocationDisplay;

void KinematicInformant::updateRobotFrames() {
    // Update IMU Stuff
    updateIMUKinematicStateVector();

    // Update Chassis Stuff after IMU STUFF
    updateChassisIMUAngles();
    updateChassisAcceleration();

    chassisKFOdometry.update(
        getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians),
        chassisLinearState[X_AXIS].getAcceleration(),
        chassisLinearState[Y_AXIS].getAcceleration());

    modm::Location2D<float> robotLocation = chassisKFOdometry.getCurrentLocation2D();

    robotFrames.updateFrames(
        gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians),
        gimbalSubsystem->getCurrentPitchAxisAngle(AngleUnit::Radians),
        getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians),
        {robotLocation.getX(), robotLocation.getY(), 0},
        AngleUnit::Radians);

    robotLocationDisplay = robotLocation;
}

void KinematicInformant::mirrorPastRobotFrame(uint32_t frameDelay_ms) {
    std::pair<float, float> gimbalAngles = gimbalSubsystem->getGimbalOrientationAtTime(frameDelay_ms);

    robotFrames.mirrorPastCameraFrame(gimbalAngles.first, gimbalAngles.second, AngleUnit::Radians);
    // robotFrames.mirrorPastCameraFrame(modm::toRadian(0.0f), modm::toRadian(0.0f), AngleUnit::Radians);
}

}  // namespace src::Informants