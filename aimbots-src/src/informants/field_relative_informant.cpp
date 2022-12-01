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
      fieldRelativeRobotPosition(Matrix<float, 1, 3>::zeroMatrix())
#ifdef TARGET_SENTRY
      ,
      railRelativePosition(Matrix<float, 1, 3>::zeroMatrix())
#endif
{
}

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

tap::communication::sensors::imu::ImuInterface::ImuState FieldRelativeInformant::getImuState() { return drivers->bmi088.getImuState(); }

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

float robotPositionXDisplay = 0.0f;
float robotPositionYDisplay = 0.0f;
float robotPositionZDisplay = 0.0f;
float robotRailPositionXDisplay = 0.0f;

float railPositionRawDebug, wheelOffsetDebug;

void FieldRelativeInformant::updateFieldRelativeRobotPosition() {
#ifdef TARGET_SENTRY
    // first, get unwrapped motor position
    float motorRevolutionsUnwrapped;

    motorRevolutionsUnwrapped = static_cast<float>(odomRailMotor->getEncoderUnwrapped()) /
                                static_cast<float>(odomRailMotor->ENC_RESOLUTION);  // current position in motor revolutions

    // now, convert to unwrapped wheel revolutions
    float wheelRevolutionsUnwrapped = motorRevolutionsUnwrapped * CHASSIS_GEARBOX_RATIO;  // current position in wheel revolutions
    // now, convert to unwrapped wheel rotations to get the rail position
    float currWheelMovement = -wheelRevolutionsUnwrapped * (2.0f * M_PI * WHEEL_RADIUS) + wheelOffset;  // current position in meters

    // grab ultrasonic data, recalibrate wheels if ultrasonics valid

    // Ultrasonic currently unused
    // float currUltrasonicPosition = UltrasonicDistanceSensor::getRailPosition() / 100.0;  // ultrasonic position in meters

    if (UltrasonicDistanceSensor::isDataValid()) {
        // wheelOffset += (currUltrasonicPosition - currWheelMovement) * ULTRASONIC_TRUST_FACTOR;
    }

    railPositionRawDebug = ((UltrasonicDistanceSensor::getRightDistance() + UltrasonicDistanceSensor::getLeftDistance()) / 2.0) / 100.0;  // meters
    wheelOffsetDebug = wheelOffset;

    // set the current rail position to a position matrix relative to the rail
    railRelativePosition[0][0] = currWheelMovement + robot_starting_rail_location_array[0];
    robotRailPositionXDisplay = railRelativePosition[0][0];

    // rotate the matrix by 45 degrees (rail is mounted at 45 degree angle) and add to the robot's starting position
    fieldRelativeRobotPosition =
        railRelativePosition * src::utils::MatrixHelper::rotation_matrix(AngleUnit::Degrees, 45.0f, 2) + left_sentry_rail_pole_location_matrix;

    robotPositionXDisplay = fieldRelativeRobotPosition[0][0];
    robotPositionYDisplay = fieldRelativeRobotPosition[0][1];
    robotPositionZDisplay = fieldRelativeRobotPosition[0][2];
#endif
    // Step 1. get the CenterAccMatrix.
    // CenterAccMatrix(3x1) - ImuAccMatrix(3x1)
    //    = alpha(3x1) x (r_center - r_imu) + omega(3x1) x omega(3x1) x (r_center - r_imu)

    // alpha: Imu angular acceleration
    // omega: Imu angular velocity

    Matrix<float, 3, 1> CenterAccMatrix = Matrix<float, 3, 1>::zeroMatrix();
    Matrix<float, 3, 1> ImuAccMatrix = Matrix<float, 3, 1>::zeroMatrix();
    Matrix<float, 3, 1> alphaMatrix = Matrix<float, 3, 1>::zeroMatrix();
    Matrix<float, 3, 1> omegaMatrix = Matrix<float, 3, 1>::zeroMatrix();

    // need this from hardware info.
    Matrix<float, 3, 1> imuToCenterMatrix = Matrix<float, 3, 1>::zeroMatrix();

    ImuAccMatrix[0][0] = drivers->bmi088.getAx();
    ImuAccMatrix[1][0] = drivers->bmi088.getAy();
    ImuAccMatrix[2][0] = drivers->bmi088.getAz();
    
    CenterAccMatrix = ImuAccMatrix;

    // alphaMatrix[0][0] = drivers->imu.nxpAlgorithm.getAlphaX();
    // alphaMatrix[1][0] = drivers->imu.nxpAlgorithm.getAlphaY();
    // alphaMatrix[2][0] = drivers->imu.nxpAlgorithm.getAlphaZ();

    omegaMatrix[0][0] = drivers->imu.getOmegaX();
    omegaMatrix[1][0] = drivers->imu.getOmegaY();
    omegaMatrix[2][0] = drivers->imu.getOmegaZ();

    // cross product : alpha (angular acceleration) x (imuToCenterMatrix)
    CenterAccMatrix[0][0] += (alphaMatrix[1][0] * imuToCenterMatrix[2][0] - alphaMatrix[2][0] * imuToCenterMatrix[1][0]);
    CenterAccMatrix[1][0] += -(alphaMatrix[0][0] * imuToCenterMatrix[2][0] - alphaMatrix[2][0] * imuToCenterMatrix[0][0]);
    CenterAccMatrix[2][0] += (alphaMatrix[0][0] * imuToCenterMatrix[1][0] - alphaMatrix[1][0] * imuToCenterMatrix[0][0]);
    
    // cross product : omega (angular velocity) x ( omega (angular velocity) x (imuToCenterMatrix))
    Matrix<float, 3, 1> tmp1Matrix = Matrix<float, 3, 1>::zeroMatrix();
    Matrix<float, 3, 1> tmp2Matrix = Matrix<float, 3, 1>::zeroMatrix();

    tmp1Matrix[0][0] += (omegaMatrix[1][0] * imuToCenterMatrix[2][0] - omegaMatrix[2][0] * imuToCenterMatrix[1][0]);
    tmp1Matrix[1][0] += -(omegaMatrix[0][0] * imuToCenterMatrix[2][0] - omegaMatrix[2][0] * imuToCenterMatrix[0][0]);
    tmp1Matrix[2][0] += (omegaMatrix[0][0] * imuToCenterMatrix[1][0] - omegaMatrix[1][0] * imuToCenterMatrix[0][0]);

    tmp2Matrix[0][0] += (omegaMatrix[1][0] * tmp1Matrix[2][0] - omegaMatrix[2][0] * tmp1Matrix[1][0]);
    tmp2Matrix[1][0] += -(omegaMatrix[0][0] * tmp1Matrix[2][0] - omegaMatrix[2][0] * tmp1Matrix[0][0]);
    tmp2Matrix[2][0] += (omegaMatrix[0][0] * tmp1Matrix[1][0] - omegaMatrix[1][0] * tmp1Matrix[0][0]);

    CenterAccMatrix += tmp2Matrix;

    // Step 2. get Rotation matrix (R) x CenterAccMatrix
    Matrix<float, 3, 3> RotationMatrix = Matrix<float, 3, 3>::zeroMatrix();
    // RotationMatrix = getRotationMatrix();
    float deltaTime = 1;
    // deltaTime = getDeltaTime(); // ???what is the delta time?
    
    Matrix<float, 3, 1> locationChange = Matrix<float, 3, 1>::zeroMatrix();
    locationChange = RotationMatrix * CenterAccMatrix;

    locationChange = 0.5 * locationChange * deltaTime * deltaTime;
    
    // Step3. update position.
    // current position + locationChange;
    fieldRelativeRobotPosition[0][0] += locationChange[0][0];
    fieldRelativeRobotPosition[0][1] += locationChange[1][0];
    fieldRelativeRobotPosition[0][2] += locationChange[2][0];

    robotPositionXDisplay = fieldRelativeRobotPosition[0][0];
    robotPositionYDisplay = fieldRelativeRobotPosition[0][1];
    robotPositionZDisplay = fieldRelativeRobotPosition[0][2];
}

// gets the angle between the robot's current position and the field coordinate
float FieldRelativeInformant::getXYAngleToFieldCoordinate(AngleUnit unit, Matrix<float, 1, 3> fieldCoordinate) {
    float xy_angle = src::utils::MatrixHelper::xy_angle_between_locations(AngleUnit::Radians, fieldRelativeRobotPosition, fieldCoordinate);
    if (unit == AngleUnit::Degrees) {
        xy_angle = modm::toDegree(xy_angle);
    }
    return xy_angle;
}
}  // namespace src::Informants
