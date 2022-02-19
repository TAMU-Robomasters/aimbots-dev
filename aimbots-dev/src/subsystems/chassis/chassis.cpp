#include "subsystems/chassis/chassis.hpp"

#include <drivers.hpp>

#include "tap/communication/gpio/leds.hpp"
#include "utils/common_types.hpp"

using namespace tap::algorithms;

namespace src::Chassis {

// Allows user to call a DJIMotor member function on all chassis motors
template <class... Args>
void ChassisSubsystem::ForAllChassisMotors(void (DJIMotor::*func)(Args...), Args... args) {
    for (auto i = 0; i < DRIVEN_WHEEL_COUNT; i++) {
        (motors[i][0]->*func)(args...);
#ifdef SWERVE
        (motors[i][1]->*func)(args...);
#endif
    }
}

// Allows user to call function for all chassis motors that takes the wheel index, wheel_per_motor index, and any arguments
// For a non-DJIMotor member function to be usable by ForAllChassisMotors, it must take two integers as arguments so it can identify the wheel and motor
template <class... Args>
void ChassisSubsystem::ForAllChassisMotors(void (ChassisSubsystem::*func)(int, int, Args...), Args... args) {
    for (auto i = 0; i < DRIVEN_WHEEL_COUNT; i++) {
        (this->*func)(i, 0, args...);
#ifdef SWERVE
        (this->*func)(i, 1, args...);
#endif
    }
}

ChassisSubsystem::ChassisSubsystem(
    tap::Drivers* drivers)
    : ChassisSubsystemInterface(drivers),
#ifdef TARGET_SENTRY
      railWheel(drivers, RAIL_WHEEL_ID, CHAS_BUS, false, "Rail Motor"),
#else
      leftBackWheel(drivers, LEFT_BACK_WHEEL_ID, CHAS_BUS, false, "Left Back Wheel Motor"),
      leftFrontWheel(drivers, LEFT_FRONT_WHEEL_ID, CHAS_BUS, false, "Left Front Wheel Motor"),
      rightFrontWheel(drivers, RIGHT_FRONT_WHEEL_ID, CHAS_BUS, false, "Right Front Wheel Motor"),
      rightBackWheel(drivers, RIGHT_BACK_WHEEL_ID, CHAS_BUS, false, "Right Back Wheel Motor"),
#ifdef SWERVE
      leftBackYaw(drivers, LEFT_BACK_YAW_ID, CHAS_BUS, false, "Left Back Yaw Motor"),
      leftFrontYaw(drivers, LEFT_FRONT_YAW_ID, CHAS_BUS, false, "Left Front Yaw Motor"),
      rightFrontYaw(drivers, RIGHT_FRONT_YAW_ID, CHAS_BUS, false, "Right Front Yaw Motor"),
      rightBackYaw(drivers, RIGHT_BACK_YAW_ID, CHAS_BUS, false, "Right Back Yaw Motor"),
#endif
#endif
      targetRPMs(Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
      motors(Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
      velocityPIDs(Matrix<StockPID*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
      wheelLocationMatrix(Matrix<float, 4, 3>::zeroMatrix())
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
    setDesiredOutputs(0, 0, 0);
}

void ChassisSubsystem::refresh() {
    // update motor rpm based on the robot type?
    ForAllChassisMotors(&ChassisSubsystem::updateMotorVelocityPID);
}

void ChassisSubsystem::updateMotorVelocityPID(int WheelIdx, int motorPerWheelIdx) {
    float err = targetRPMs[WheelIdx][motorPerWheelIdx] - motors[WheelIdx][motorPerWheelIdx]->getShaftRPM();
    velocityPIDs[WheelIdx][motorPerWheelIdx]->update(err);

    motors[WheelIdx][motorPerWheelIdx]->setDesiredOutput(velocityPIDs[WheelIdx][motorPerWheelIdx]->getValue());
}

void ChassisSubsystem::setDesiredOutputs(float x, float y, float r) {
#if defined(TARGET_SENTRY)
    calculateRail(x);
#elif defined(SWERVE)
    calculateSwerve(x, y, r,
                    ChassisSubsystem::getMaxUserWheelSpeed(drivers->refSerial.getRefSerialReceivingData(), drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
#else
    calculateMecanum(x, y, r,
                     ChassisSubsystem::getMaxUserWheelSpeed(drivers->refSerial.getRefSerialReceivingData(), drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
#endif
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
        y + x + chassisRotateTranslated * leftFrontRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    targetRPMs[RF][0] = limitVal<float>(
        y - x + chassisRotateTranslated * rightFrontRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    targetRPMs[LB][0] = limitVal<float>(
        -y + x + chassisRotateTranslated * leftBackRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    targetRPMs[RB][0] = limitVal<float>(
        -y - x + chassisRotateTranslated * rightBackRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);

    desiredRotation = r;
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

void ChassisSubsystem::calculateSwerve(float, float, float, float) {}

void ChassisSubsystem::calculateRail(float) {}

};  // namespace src::Chassis