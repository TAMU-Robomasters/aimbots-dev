#include "field_relative_informant.hpp"

#include "subsystems/gimbal/gimbal.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Informants {

FieldRelativeInformant::FieldRelativeInformant(src::Drivers* drivers)
    : drivers(drivers),
      gimbal(nullptr),
      robotStartingPosition(ROBOT_STARTING_POSITION),
      fieldRelativeRobotPosition(Matrix<float, 1, 3>::zeroMatrix()) {}

void FieldRelativeInformant::initialize(float imuSampleFrequency, float imukP, float imukI) {
    drivers->bmi088.initialize(imuSampleFrequency, imukP, imukI);
}

// ! Dev Board must be level during IMU calibration
void FieldRelativeInformant::recalibrateIMU() { drivers->bmi088.requestRecalibration(); }

inline float FieldRelativeInformant::getCurrentFieldRelativeGimbalYaw(AngleUnit unit) const {
    return (gimbal) ? gimbal->getCurrentFieldRelativeYawAngle(unit) : 0.0f;
}

// ? positive yaw is clockwise viewed top-down
float FieldRelativeInformant::getChassisYaw() {
    // rotates by 180 degrees and negates the result, to force a 0 starting angle and postitive clockwise
    return -modm::toRadian(drivers->bmi088.getYaw() - 180.0f);
}

// ? positive pitch is clockwise viewed from left side of dev board
float FieldRelativeInformant::getChassisPitch() {
    // rotates pitch/roll vectors to account for yaw offset in dev board mounting
    float pitch = modm::toRadian(drivers->bmi088.getPitch());
    float roll = modm::toRadian(drivers->bmi088.getRoll());
    tap::algorithms::rotateVector(&pitch, &roll, getChassisYaw() + DEV_BOARD_YAW_OFFSET);
    return -pitch;
}

// ? positive roll is clockwise viewed from front side of dev board
float FieldRelativeInformant::getChassisRoll() {
    // rotates pitch/roll vectors to account for yaw offset in dev board mounting
    float pitch = modm::toRadian(drivers->bmi088.getPitch());
    float roll = modm::toRadian(drivers->bmi088.getRoll());
    tap::algorithms::rotateVector(&pitch, &roll, getChassisYaw() + DEV_BOARD_YAW_OFFSET);
    return roll;
}

tap::communication::sensors::imu::ImuInterface::ImuState FieldRelativeInformant::getImuState() {
    return drivers->bmi088.getImuState();
}

float FieldRelativeInformant::getGz() {  // yaw
    return -modm::toRadian(drivers->bmi088.getGz());
}
float FieldRelativeInformant::getGy() {  // pitch
    float gY = modm::toRadian(drivers->bmi088.getGy());
    float gX = modm::toRadian(drivers->bmi088.getGx());
    tap::algorithms::rotateVector(&gY, &gX, getChassisYaw() + DEV_BOARD_YAW_OFFSET);
    return -gY;
}
float FieldRelativeInformant::getGx() {  // roll
    float gY = modm::toRadian(drivers->bmi088.getGy());
    float gX = modm::toRadian(drivers->bmi088.getGx());
    tap::algorithms::rotateVector(&gY, &gX, getChassisYaw() + DEV_BOARD_YAW_OFFSET);
    return gX;
}
float FieldRelativeInformant::getAz() { return drivers->bmi088.getAz(); }
float FieldRelativeInformant::getAy() { return drivers->bmi088.getAy(); }
float FieldRelativeInformant::getAx() { return drivers->bmi088.getAx(); }

void FieldRelativeInformant::updateFieldRelativeRobotPosition() {}
}  // namespace src::Informants