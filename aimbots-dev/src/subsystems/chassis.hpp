#pragma once

#include "modm/math/matrix.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "utils/common_types.hpp"

#include "utils/robot_constants.hpp"

namespace Chassis {

class ChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface {
   public:
    void calculateMecanum(float x, float y, float r);  // normal 4wd mecanum robots
    void calculateSwerve(float x, float y, float r);   // swerve drive robots
    void calculateRail(float x);                       // sentry rail robots

   private:
    modm::Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> targetRPMs;

    modm::Matrix<DJIMotor, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> motors;

    ChassisPowerLimiter powerLimiter;

    static constexpr CANBus CAN_BUS = CANBus::CAN_BUS2;
};
};  // namespace Chassis