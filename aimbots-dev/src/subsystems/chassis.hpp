#pragma once

#include "modm/math/matrix.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "utils/common_types.hpp"

// #include "robots/aerial/aerial_constants.hpp"
#include "robots/engineer/engineer_constants.hpp"
#include "robots/engineer/swerve_engineer_constants.hpp"
#include "robots/hero/hero_constants.hpp"
#include "robots/sentry/sentry_constants.hpp"
#include "robots/standard/standard_constants.hpp"
#include "robots/standard/swerve_standard_constants.hpp"

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