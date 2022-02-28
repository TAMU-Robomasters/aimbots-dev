#pragma once

#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Chassis {

<<<<<<< HEAD
    enum WheelIndex {  // index used to easily navigate wheel matrices
        RAIL = 0,      //	 ___     ___ 	          __
        LB = 0,        //	| 1 |___| 2 |	    | ___ __|
        LF = 1,        //	|___| | |___|	    |  | |__
        RF = 2,        //	      |      	       |
        RB = 3         //	 ___  |  ___ 	   __  |  __
                       //	| 0 |_|_| 3 |	  | /|_|_ __|
                       //	|___|   |___|	  |/_|    __|
    };
=======
enum WheelIndex {  // index used to easily navigate wheel matrices
    RAIL = 0,      //	 ___     ___ 	          __
    LB = 0,        //	| 1 |___| 2 |	    | ___ __|
    LF = 1,        //	|___| | |___|	    |  | |__
    RF = 2,        //	      |      	       |
    RB = 3         //	 ___  |  ___ 	   __  |  __
                   //	| 0 |_|_| 3 |	  |  |_|_ __|
                   //	|___|   |___|	  |__|    __|
};
>>>>>>> subsystems/chassis

class ChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface {
   public:
    ChassisSubsystem(  // Default chassis constructor
        tap::Drivers* drivers);

    template <class... Args>
    void ForAllChassisMotors(void (DJIMotor::*func)(Args...), Args... args);

    template <class... Args>
    void ForAllChassisMotors(void (ChassisSubsystem::*func)(int, int, Args...), Args... args);

    mockable void initialize() override;
    void refresh() override;

    void updateMotorVelocityPID(int WheelIdx, int motorPerWheelIdx);

    /**
     * @brief Updates the desired wheel RPM based on the input x, y, and rotation movement components.
     *
     * @param[in] x The desired velocity of the wheels to move in the X (horizontal) direction.
     * If x = 1000, the chassis will attempt to move the wheels forward at 1000 RPM.
     * @param[in] y The desired velocity of the wheels to move in the Y (vertical) direction.
     * See x for more information.
     * @param[in] r The desired velocity of the wheels to rotate.
     * See x for more information.
     */
    void setDesiredOutputs(float x, float y, float r);

    void calculateMecanum(float x, float y, float r, float maxWheelSpeed);  // normal 4wd mecanum robots
    void calculateSwerve(float x, float y, float r, float maxWheelSpeed);   // swerve drive robots
    void calculateRail(float x);                                            // sentry rail robots

    inline int getNumChassisMotors() const override { return DRIVEN_WHEEL_COUNT * MOTORS_PER_WHEEL; }

    /**
     * @return A number between 0 and 1 that is the ratio between the rotationRpm and
     *      the max rotation speed.
     */
    mockable float calculateRotationTranslationalGain(float chassisRotationDesiredWheelspeed);

    static inline float getMaxUserWheelSpeed(bool refSerialOnline, int chassisPower) {
        if (refSerialOnline) {
            float desWheelSpeed = WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE *
                                      static_cast<float>(chassisPower - MIN_CHASSIS_POWER) +
                                  MIN_WHEEL_SPEED_SINGLE_MOTOR;

            return tap::algorithms::limitVal(
                desWheelSpeed,
                static_cast<float>(MIN_WHEEL_SPEED_SINGLE_MOTOR),
                static_cast<float>(MAX_WHEEL_SPEED_SINGLE_MOTOR));
        } else {
            return MIN_WHEEL_SPEED_SINGLE_MOTOR;
        }
    }

#ifndef ENV_UNIT_TESTS
   private:
#else
   public:
#endif
    static constexpr CANBus CHAS_BUS = CANBus::CAN_BUS2;

#ifdef TARGET_SENTRY
    DJIMotor railWheel;
    StockPID railWheelVelPID;
#else
    DJIMotor leftBackWheel, leftFrontWheel, rightFrontWheel, rightBackWheel;
    StockPID leftBackWheelVelPID, leftFrontWheelVelPID, rightFrontWheelVelPID, rightBackWheelVelPID;
    float desiredRotation = 0.0f;
#ifdef SWERVE
    DJIMotor leftBackYaw, leftFrontYaw, rightFrontYaw, rightBackYaw;
    StockPID leftBackYawPosPID, leftFrontYawPosPID, rightFrontYawPosPID, rightBackYawPosPID;
#endif
#endif

    Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> targetRPMs;

    Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> motors;

    // ChassisPowerLimiter powerLimiter;

    Matrix<StockPID*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> velocityPIDs;

    Matrix<float, 4, 3> wheelLocationMatrix;

   public:
#ifdef TARGET_SENTRY
    inline int16_t
    getLeftFrontRpmActual() const override {
        return railWheel.getShaftRPM();
    }
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