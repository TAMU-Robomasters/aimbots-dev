#pragma once

#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace Chassis {

enum WheelIndex {  // index used to easily navigate wheel matrices
    RAIL = 0,
    LB = 0,
    LF = 1,
    RF = 2,
    RB = 3,
};

class ChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface {
   public:
    ChassisSubsystem(  // Default chassis constructor
        tap::Drivers* drivers);

    template <class... Args>
    void ForChassisMotors(void (DJIMotor::*func)(Args...), Args... args);

    void initialize() override;
    void refresh() override;

    void calculateMecanum(float x, float y, float r);  // normal 4wd mecanum robots
    void calculateSwerve(float x, float y, float r);   // swerve drive robots
    void calculateRail(float x);                       // sentry rail robots

   private:
    static constexpr CANBus CHAS_BUS = CANBus::CAN_BUS2;

    DJIMotor leftBackWheel, leftFrontWheel, rightFrontWheel, rightBackWheel;
#ifdef SWERVE
    DJIMotor leftBackYaw, leftFrontYaw, rightFrontYaw, rightBackYaw;
#endif

    Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> targetRPMs;

    Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> motors;

    // ChassisPowerLimiter powerLimiter;
};

};  // namespace Chassis