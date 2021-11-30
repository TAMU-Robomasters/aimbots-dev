#pragma once

#include "modm/math/matrix.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "utils/common_types.hpp"

namespace Chassis {

class ChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface {
   public:
    void calculateMecanum(float x, float y, float r);  // normal 4wd mecanum robots
    void calculateSwerve(float x, float y, float r);   // swerve drive robots
    void calculateRail(float x);                       // sentry rail robots

   private:
#if defined TARGET_STANDARD
#include "robots/standard/standard_constants.hpp"
    modm::Matrix<float, 4, 1> targetWheelRPMs;
#elif defined TARGET_HERO
#include "robots/hero/hero_constants.hpp"
    modm::Matrix<float, 4, 1> targetWheelRPMs;
#elif defined TARGET_ENGINEER
#include "robots/engineer/engineer_constants.hpp"
    modm::Matrix<float, 4, 1> targetWheelRPMs;
#elif defined TARGET_SENTRY
#include "robots/sentry/sentry_constants.hpp"
    modm::Matrix<float, 1, 1> targetWheelRPMs;
#elif defined TARGET_AERIAL
#include "robots/aerial/aerial_constants.hpp"
    // modm::Matrix<float, 1, 1> targetWheelRPMs;
#elif defined TARGET_SWERVE_STANDARD
#include "robots/standard/swerve_standard_constants.hpp"
    modm::Matrix<float, 4, 2> targetWheelRPMs;
#elif defined TARGET_SWERVE_ENGINEER
#include "robots/engineer/swerve_engineer_constants.hpp"
    modm::Matrix<float, 4, 2> targetWheelRPMs;
#else
#include "robots/standard/standard_constants.hpp"
    modm::Matrix<float, 4, 1> targetWheelRPMs;
#endif
};
};  // namespace Chassis