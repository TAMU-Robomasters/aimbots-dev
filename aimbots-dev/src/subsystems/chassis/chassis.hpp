#pragma once

#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Chassis {

enum WheelIndex {  // index used to easily navigate wheel matrices
    RAIL = 0,      //	 ___     ___ 	          __
    LB = 0,        //	| 1 |___| 2 |	    | ___ __|
    LF = 1,        //	|___| | |___|	    |  | |__
    RF = 2,        //	      |      	       |
    RB = 3         //	 ___  |  ___ 	   __  |  __
                   //	| 0 |_|_| 3 |	  |  |_|_ __|
                   //	|___|   |___|	  |__|    __|
};

class ChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface {
   public:
    ChassisSubsystem(  // Default chassis constructor
        tap::Drivers* drivers);

    template <class... Args>
    void ForChassisMotors(void (DJIMotor::*func)(Args...), Args... args);

    mockable void initialize() override;
    void refresh() override;

    void calculateMecanum(float x, float y, float r);  // normal 4wd mecanum robots
    void calculateSwerve(float x, float y, float r);   // swerve drive robots
    void calculateRail(float x);                       // sentry rail robots

    inline int getNumChassisMotors() const override { return DRIVEN_WHEEL_COUNT * MOTORS_PER_WHEEL; }

#ifndef ENV_UNIT_TESTS
   private:
#else
   public:
#endif
    static constexpr CANBus CHAS_BUS = CANBus::CAN_BUS2;

#ifdef TARGET_SENTRY
    DJIMotor railWheel;
#else
    DJIMotor leftBackWheel, leftFrontWheel, rightFrontWheel, rightBackWheel;
#ifdef SWERVE
    DJIMotor leftBackYaw, leftFrontYaw, rightFrontYaw, rightBackYaw;
#endif
#endif

    Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> targetRPMs;

    Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> motors;

    // ChassisPowerLimiter powerLimiter;

   public:
#ifdef TARGET_SENTRY
    inline int16_t getLeftFrontRpmActual() const override { return railWheel.getShaftRPM(); }
    inline int16_t getLeftBackRpmActual() const override { return railWheel.getShaftRPM(); }
    inline int16_t getRightFrontRpmActual() const override { return railWheel.getShaftRPM(); }
    inline int16_t getRightBackRpmActual() const override { return railWheel.getShaftRPM(); }
#else
    inline int16_t getLeftFrontRpmActual() const override { return leftFrontWheel.getShaftRPM(); }
    inline int16_t getLeftBackRpmActual() const override { return leftBackWheel.getShaftRPM(); }
    inline int16_t getRightFrontRpmActual() const override { return rightFrontWheel.getShaftRPM(); }
    inline int16_t getRightBackRpmActual() const override { return rightBackWheel.getShaftRPM(); }
#endif
};

};  // namespace src::Chassis