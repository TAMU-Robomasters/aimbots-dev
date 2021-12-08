#include "subsystems/chassis.hpp"

#include <functional>

namespace Chassis {

template <class... Args>
void ChassisSubsystem::ForChassisMotors(void (DJIMotor::*func)(Args...), Args... args) {
    for (auto i = 0; i < DRIVEN_WHEEL_COUNT; i++) {
        (motors[i][0]->*func)(args...);
#ifdef SWERVE
        (motors[i][1]->*func)(args...);
#endif
    }
}

ChassisSubsystem::ChassisSubsystem(
    tap::Drivers* drivers)
    : ChassisSubsystemInterface(drivers),
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
      targetRPMs(Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
      motors(Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix())
//
{
    motors[LB][0] = &leftBackWheel;
    motors[LF][0] = &leftFrontWheel;
    motors[RF][0] = &rightFrontWheel;
    motors[RB][0] = &rightBackWheel;
#ifdef SWERVE
    motors[LB][1] = &leftBackYaw;
    motors[LF][1] = &leftFrontYaw;
    motors[RF][1] = &rightFrontYaw;
    motors[RB][1] = &rightBackYaw;
#endif
}

void ChassisSubsystem::initialize() {
    // ForChassisMotors(&DJIMotor::initialize);
    // ForChassisMotors<int32_t>(&DJIMotor::setDesiredOutput, 0);
}

void ChassisSubsystem::refresh() {
    // update motor rpm based on the robot type?
}

void ChassisSubsystem::calculateMecanum(float, float, float) {}

void ChassisSubsystem::calculateSwerve(float, float, float) {}

void ChassisSubsystem::calculateRail(float) {}

};  // namespace Chassis