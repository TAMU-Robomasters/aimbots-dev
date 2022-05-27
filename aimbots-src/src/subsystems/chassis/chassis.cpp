#include "subsystems/chassis/chassis.hpp"

#include <drivers.hpp>

#include "tap/communication/gpio/leds.hpp"
#include "utils/common_types.hpp"

using namespace tap::algorithms;

namespace src::Chassis {

ChassisSubsystem::ChassisSubsystem(
    tap::Drivers* drivers) : ChassisSubsystemInterface(drivers),
#ifdef TARGET_SENTRY
                             railWheel(drivers, RAIL_WHEEL_ID, CHASSIS_BUS, false, "Rail Motor"),
                             railWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
#else
                             leftBackWheel(drivers, LEFT_BACK_WHEEL_ID, CHASSIS_BUS, false, "Left Back Wheel Motor"),
                             leftFrontWheel(drivers, LEFT_FRONT_WHEEL_ID, CHASSIS_BUS, false, "Left Front Wheel Motor"),
                             rightFrontWheel(drivers, RIGHT_FRONT_WHEEL_ID, CHASSIS_BUS, false, "Right Front Wheel Motor"),
                             rightBackWheel(drivers, RIGHT_BACK_WHEEL_ID, CHASSIS_BUS, false, "Right Back Wheel Motor"),

                             leftBackWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
                             leftFrontWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
                             rightFrontWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
                             rightBackWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),

#ifdef SWERVE
                             leftBackYaw(drivers, LEFT_BACK_YAW_ID, CHASSIS_BUS, false, "Left Back Yaw Motor"),
                             leftFrontYaw(drivers, LEFT_FRONT_YAW_ID, CHASSIS_BUS, false, "Left Front Yaw Motor"),
                             rightFrontYaw(drivers, RIGHT_FRONT_YAW_ID, CHASSIS_BUS, false, "Right Front Yaw Motor"),
                             rightBackYaw(drivers, RIGHT_BACK_YAW_ID, CHASSIS_BUS, false, "Right Back Yaw Motor"),
#endif
#endif
                             targetRPMs(Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
                             desiredOutputs(Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
                             motors(Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
                             velocityPIDs(Matrix<SmoothPID*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
                             wheelLocationMatrix(Matrix<float, 4, 3>::zeroMatrix()),
                             robotPositionFieldRelative(Matrix<float, 1, 3>::zeroMatrix())
//
{
#ifdef TARGET_SENTRY
    motors[RAIL][0] = &railWheel;
    velocityPIDs[RAIL][0] = &railWheelVelPID;
#else
    motors[LB][0] = &leftBackWheel;
    motors[LF][0] = &leftFrontWheel;
    motors[RF][0] = &rightFrontWheel;
    motors[RB][0] = &rightBackWheel;

    velocityPIDs[LB][0] = &leftBackWheelVelPID;
    velocityPIDs[LF][0] = &leftFrontWheelVelPID;
    velocityPIDs[RF][0] = &rightFrontWheelVelPID;
    velocityPIDs[RB][0] = &rightBackWheelVelPID;

    wheelLocationMatrix[0][0] = -1.0f;
    wheelLocationMatrix[0][1] = 1.0f;
    wheelLocationMatrix[0][2] = WHEELBASE_WIDTH + WHEELBASE_LENGTH;
    wheelLocationMatrix[1][0] = 1.0f;
    wheelLocationMatrix[1][1] = 1.0f;
    wheelLocationMatrix[1][2] = -(WHEELBASE_WIDTH + WHEELBASE_LENGTH);
    wheelLocationMatrix[2][0] = -1.0f;
    wheelLocationMatrix[2][1] = 1.0f;
    wheelLocationMatrix[2][2] = WHEELBASE_WIDTH + WHEELBASE_LENGTH;
    wheelLocationMatrix[3][0] = 1.0f;
    wheelLocationMatrix[3][1] = 1.0f;
    wheelLocationMatrix[3][2] = -(WHEELBASE_WIDTH + WHEELBASE_LENGTH);

// NON SWERVE ROBOTS
#ifdef SWERVE
    // SWERVE ROBOTS
    motors[LB][1] = &leftBackYaw;
    motors[LF][1] = &leftFrontYaw;
    motors[RF][1] = &rightFrontYaw;
    motors[RB][1] = &rightBackYaw;

    velocityPIDs[LB][1] = &leftBackYawPosPID;
    velocityPIDs[LF][1] = &leftFrontYawPosPID;
    velocityPIDs[RF][1] = &rightFrontYawPosPID;
    velocityPIDs[RB][1] = &rightBackYawPosPID;
#endif
#endif
}

void ChassisSubsystem::initialize() {
    ForAllChassisMotors(&DJIMotor::initialize);
    setTargetRPMs(0, 0, 0);
}

void ChassisSubsystem::refresh() {
    // update motor rpm based on the robot type?
    updateFieldRelativePosition();

    ForAllChassisMotors(&ChassisSubsystem::updateMotorVelocityPID);

    ForAllChassisMotors(&ChassisSubsystem::setDesiredOutput);
}

void ChassisSubsystem::updateMotorVelocityPID(WheelIndex WheelIdx, MotorOnWheelIndex MotorPerWheelIdx) {
    float err = targetRPMs[WheelIdx][MotorPerWheelIdx] - motors[WheelIdx][MotorPerWheelIdx]->getShaftRPM();
    velocityPIDs[WheelIdx][MotorPerWheelIdx]->runControllerDerivateError(err);

    desiredOutputs[WheelIdx][MotorPerWheelIdx] = velocityPIDs[WheelIdx][MotorPerWheelIdx]->getOutput();
}

void ChassisSubsystem::setTargetRPMs(float x, float y, float r) {
#if defined(TARGET_SENTRY)
    calculateRail(x,
                  ChassisSubsystem::getMaxUserWheelSpeed(drivers->refSerial.getRefSerialReceivingData(), drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
#elif defined(SWERVE)
    calculateSwerve(x, y, r,
                    ChassisSubsystem::getMaxUserWheelSpeed(drivers->refSerial.getRefSerialReceivingData(), drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
#else
    calculateMecanum(x, y, r,
                     ChassisSubsystem::getMaxUserWheelSpeed(drivers->refSerial.getRefSerialReceivingData(), drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
#endif
}

void ChassisSubsystem::setDesiredOutput(WheelIndex WheelIdx, MotorOnWheelIndex MotorPerWheelIdx) {
    motors[WheelIdx][MotorPerWheelIdx]->setDesiredOutput(static_cast<int32_t>(desiredOutputs[WheelIdx][MotorPerWheelIdx]));
}

void ChassisSubsystem::calculateMecanum(float x, float y, float r, float maxWheelSpeed) {
    // get distance from wheel to center of wheelbase
    float wheelbaseCenterDist = sqrtf(powf(WHEELBASE_WIDTH / 2.0f, 2.0f) + powf(WHEELBASE_LENGTH / 2.0f, 2.0f));

    // offset gimbal center from center of wheelbase so we rotate around the gimbal
    float leftFrontRotationRatio =
        modm::toRadian(wheelbaseCenterDist - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightFrontRotationRatio =
        modm::toRadian(wheelbaseCenterDist - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    float leftBackRotationRatio =
        modm::toRadian(wheelbaseCenterDist + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightBackRotationRatio =
        modm::toRadian(wheelbaseCenterDist + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);

    float chassisRotateTranslated = modm::toDegree(r) / wheelbaseCenterDist;

    targetRPMs[LF][0] = limitVal<float>(
        x + y + chassisRotateTranslated * leftFrontRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    targetRPMs[RF][0] = limitVal<float>(
        x - y + chassisRotateTranslated * rightFrontRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    targetRPMs[LB][0] = limitVal<float>(
        -x + y + chassisRotateTranslated * leftBackRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    targetRPMs[RB][0] = limitVal<float>(
        -x - y + chassisRotateTranslated * rightBackRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);

    desiredRotation = r;
}

void ChassisSubsystem::calculateSwerve(float, float, float, float) {}

void ChassisSubsystem::calculateRail(float x, float maxWheelSpeed) {
    targetRPMs[RAIL][0] = limitVal<float>(x, -maxWheelSpeed, maxWheelSpeed);
}

float ChassisSubsystem::calculateRotationTranslationalGain(float chassisRotationDesiredWheelspeed) {
    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain = 1.0f;

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the wheel rotation speed for chassis rotationis greater than the
    // MIN_ROTATION_THRESHOLD
    if (fabsf(chassisRotationDesiredWheelspeed) > MIN_ROTATION_THRESHOLD) {
        // power(max revolve speed - specified revolve speed, 2)
        // / power(max revolve speed, 2)
        rTranslationalGain = powf(MAX_WHEEL_SPEED_SINGLE_MOTOR + MIN_ROTATION_THRESHOLD -
                                      fabsf(chassisRotationDesiredWheelspeed) / MAX_WHEEL_SPEED_SINGLE_MOTOR,
                                  2.0f);
        rTranslationalGain = tap::algorithms::limitVal<float>(rTranslationalGain, 0.0f, 1.0f);
    }
    return rTranslationalGain;
}

float motorRevolutionsUnwrappedDisplay = 0.0f;
float wheelRevolutionsUnwrappedDisplay = 0.0f;
float currRailPositionDisplay = 0.0f;

float robotPositionXDisplay = 0.0f;
float robotPositionYDisplay = 0.0f;
float robotPositionZDisplay = 0.0f;

void ChassisSubsystem::updateFieldRelativePosition() {
// need to create transformation vector based on sensor inputs to apply to the position matrix
#ifdef TARGET_SENTRY
    // first, get rail relative position
    float motorRevolutionsUnwrapped = static_cast<float>(motors[RAIL][0]->getEncoderUnwrapped()) / static_cast<float>(motors[RAIL][0]->ENC_RESOLUTION);  // current position in motor revolutions
    float wheelRevolutionsUnwrapped = motorRevolutionsUnwrapped * CHASSIS_GEARBOX_RATIO;                                                                 // current position in wheel revolutions
    float currRailPosition = -wheelRevolutionsUnwrapped * (2 * M_PI * WHEEL_RADIUS);                                                                     // current position in meters
    motorRevolutionsUnwrappedDisplay = motorRevolutionsUnwrapped;
    wheelRevolutionsUnwrappedDisplay = wheelRevolutionsUnwrapped;
    currRailPositionDisplay = currRailPosition;
    Matrix<float, 1, 3> railRelativePosition = Matrix<float, 1, 3>::zeroMatrix();
    railRelativePosition[0][0] = currRailPosition;

    static float xy_rotation_array_45deg[9] = {
        cosf(M_PI / 4.0f), -sinf(M_PI / 4.0f), 0.0f,
        sinf(M_PI / 4.0f), cosf(M_PI / 4.0f), 0.0f,
        0.0f, 0.0f, 1.0f};

    static const Matrix<float, 3, 3> xy_rotation_matrix_45deg = Matrix<float, 3, 3>(xy_rotation_array_45deg);
    robotPositionFieldRelative = railRelativePosition * xy_rotation_matrix_45deg + ROBOT_STARTING_LOCATION;

    robotPositionXDisplay = robotPositionFieldRelative[0][0];
    robotPositionYDisplay = robotPositionFieldRelative[0][1];
    robotPositionZDisplay = robotPositionFieldRelative[0][2];

#endif
}

Matrix<float, 1, 3> ChassisSubsystem::getFieldRelativePosition() {
    return robotPositionFieldRelative;
}
};  // namespace src::Chassis
