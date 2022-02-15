#include "subsystems/chassis/chassis.hpp"

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

// Allows user to call function for all chassis motors that takes a DJIMotor and some arguments
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
      wheelLocationMatrix(Matrix<float, 4, 3>::zeroMatrix()),
      velocityPIDs(Matrix<StockPID*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix())
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
    calculateSwerve(x, y, r);
#else
    calculateMecanum(x, y, r);
#endif
}

void ChassisSubsystem::calculateMecanum(float x, float y, float r) {
}

void ChassisSubsystem::calculateSwerve(float, float, float) {}

void ChassisSubsystem::calculateRail(float) {}

};  // namespace src::Chassis