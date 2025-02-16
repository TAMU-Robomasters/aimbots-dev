#include "kinematic_informant.hpp"
#include "imu_data.hpp"

#include "drivers.hpp"

namespace src::Informants {

IMUData::IMUData(src::Drivers* drivers) : drivers(drivers) {} 
 
Vector3f IMUData::getRawLocalIMUAngles() {
    Vector3f imuAngles = {
        -drivers->bmi088.getPitch(),  // inverts pitch
        drivers->bmi088.getRoll(),
        drivers->bmi088.getYaw() - 180.0f};  // for some reason yaw is 180.0 degrees rotated
    return imuAngles * (M_PI / 180.0f);      // Convert to rad
}

float IMUData::getRawLocalIMUAngle(AngularAxis axis) {  // Gets IMU angles in IMU Frame
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

Vector3f IMUData::getRawIMUAngularVelocities() {  // Gets IMU Angular Velocity in IMU FRAME
    Vector3f imuAngularVelocities = {-drivers->bmi088.getGy(), drivers->bmi088.getGx(), drivers->bmi088.getGz()};
    return imuAngularVelocities * (M_PI / 180.0f);  // Convert to rad/s
}

float IMUData::getRawIMUAngularVelocity(AngularAxis axis) {  // Gets IMU angles in IMU Frame
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
void IMUData::updateIMUKinematicStateVector() {
    imuLinearState[X_AXIS].updateFromAcceleration(-drivers->bmi088.getAy());
    imuLinearState[Y_AXIS].updateFromAcceleration(drivers->bmi088.getAx());
    imuLinearState[Z_AXIS].updateFromAcceleration(drivers->bmi088.getAz());

    imuAngularState[X_AXIS].updateFromPosition(getRawLocalIMUAngle(PITCH_AXIS));
    imuAngularState[Y_AXIS].updateFromPosition(getRawLocalIMUAngle(ROLL_AXIS));
    imuAngularState[Z_AXIS].updateFromPosition(getRawLocalIMUAngle(YAW_AXIS));

    imuAngularState[X_AXIS].updateFromVelocity(getIMUAngularVelocity(PITCH_AXIS, AngleUnit::Radians), false);
    imuAngularState[Y_AXIS].updateFromVelocity(getIMUAngularVelocity(ROLL_AXIS, AngleUnit::Radians), false);
    imuAngularState[Z_AXIS].updateFromVelocity(getIMUAngularVelocity(YAW_AXIS, AngleUnit::Radians), false);
}

Vector3f IMUData::getIMUAngularAccelerations() {
    float alphax = imuAngularState[X_AXIS].getAcceleration();
    float alphay = imuAngularState[Y_AXIS].getAcceleration();
    float alphaz = imuAngularState[Z_AXIS].getAcceleration();

    Vector3f alpha = {alphax, alphay, alphaz};
    return alpha;
}

Vector3f IMUData::getRawIMULinearAccelerations() {
    float ax = -drivers->bmi088.getAy();
    float ay = drivers->bmi088.getAx();
    float az = drivers->bmi088.getAz();

    Vector3f a = {ax, ay, az};
    return a;
}

float IMUData::getRawIMULinearAcceleration(LinearAxis axis) {  // Gets IMU accel in IMU Frame
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

void IMUData::updateIMUAngles() {
    Vector3f IMUAngles = getRawLocalIMUAngles();
    Vector3f IMUAngularVelocities = getRawIMUAngularVelocities();

    IMUAnglesDisplay = IMUAngles;

    // Gets chassis angles
    Vector3f chassisAngles =
        drivers->kinematicInformant.fieldRelativeGimbal.getRobotFrames()
            .getFrame(Transformers::FrameType::CHASSIS_IMU_FRAME)
            .getPointInFrame(
                drivers->kinematicInformant.fieldRelativeGimbal.getRobotFrames().getFrame(Transformers::FrameType::CHASSIS_FRAME),
                IMUAngles);

    chassisAnglesConvertedDisplay = chassisAngles;

    Vector3f chassisAngularVelocities =
        drivers->kinematicInformant.fieldRelativeGimbal.getRobotFrames()
            .getFrame(Transformers::FrameType::CHASSIS_IMU_FRAME)
            .getPointInFrame(
                drivers->kinematicInformant.fieldRelativeGimbal.getRobotFrames().getFrame(Transformers::FrameType::CHASSIS_FRAME),
                IMUAngularVelocities);

    chassisAngularState[X_AXIS].updateFromPosition(chassisAngles[X_AXIS]);
    chassisAngularState[Y_AXIS].updateFromPosition(chassisAngles[Y_AXIS]);
    chassisAngularState[Z_AXIS].updateFromPosition(chassisAngles[Z_AXIS]);

    chassisAngularState[X_AXIS].updateFromVelocity(chassisAngularVelocities[X_AXIS], false);
    chassisAngularState[Y_AXIS].updateFromVelocity(chassisAngularVelocities[Y_AXIS], false);
    chassisAngularState[Z_AXIS].updateFromVelocity(chassisAngularVelocities[Z_AXIS], false);
}
float angleDisplay = -1;
float IMUData::getIMUAngle(AngularAxis axis, AngleUnit unit) {
    float angle = chassisAngularState[axis].getPosition();
    angleDisplay = angle;
    return unit == AngleUnit::Radians ? angle : modm::toDegree(angle);
}

float IMUData::getIMUAngularVelocity(AngularAxis axis, AngleUnit unit) {
    float angularVelocity = chassisAngularState[axis].getVelocity();

    return unit == AngleUnit::Radians ? angularVelocity : modm::toDegree(angularVelocity);
}

float IMUData::getIMUAngularAcceleration(AngularAxis axis, AngleUnit unit) {
    float angularAcceleration = imuAngularState[axis].getAcceleration();

    return unit == AngleUnit::Radians ? angularAcceleration : modm::toDegree(angularAcceleration);
}

tap::communication::sensors::imu::ImuInterface::ImuState IMUData::getIMUState() {
    return drivers->bmi088.getImuState();
}

void IMUData::recalibrateIMU(Vector3f imuCalibrationEuler) {
    // drivers->bmi088.requestRecalibration(imuCalibrationEuler);
    UNUSED(imuCalibrationEuler);
    drivers->bmi088.requestRecalibration();
};

}  // namespace Informants

