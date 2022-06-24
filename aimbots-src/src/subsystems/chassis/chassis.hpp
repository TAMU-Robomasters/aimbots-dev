#pragma once

#include "drivers.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "utils/common_types.hpp"
#include "utils/motion/power_limiter/power_limiter.hpp"
#include "utils/robot_specific_inc.hpp"

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

enum MotorOnWheelIndex {
    DRIVER = 0,  // in this case driver indicates the motor rotating the drive wheels
    YAW = 1
};

class ChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface {
   public:
    ChassisSubsystem(  // Default chassis constructor
        src::Drivers* drivers);

    /**
     * Allows user to call a DJIMotor member function on all chassis motors
     *
     * @param function pointer to a member function of DJIMotor
     * @param args arguments to pass to the member function
     */
    template <class... Args>
    void ForAllChassisMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto i = 0; i < DRIVEN_WHEEL_COUNT; i++) {
            (motors[i][0]->*func)(args...);
#ifdef SWERVE
            (motors[i][1]->*func)(args...);
#endif
        }
    }

    /**
     * Allows user to call a ChassisSubsystem function on all chassis motors.
     *
     * @param function pointer to a member function of ChassisSubsystem that takes a WheelIndex
     * as its first argument and the motor-per-wheel index as the second argument
     * @param args arguments to pass to the member function
     */
    template <class... Args>
    void ForAllChassisMotors(void (ChassisSubsystem::*func)(WheelIndex, MotorOnWheelIndex, Args...), Args... args) {
        for (auto i = 0; i < DRIVEN_WHEEL_COUNT; i++) {
            WheelIndex mi = static_cast<WheelIndex>(i);
            (this->*func)(mi, DRIVER, args...);
#ifdef SWERVE
            (this->*func)(mi, YAW, args...);
#endif
        }
    }

    mockable void initialize() override;
    void refresh() override;

    void updateMotorVelocityPID(WheelIndex WheelIdx, MotorOnWheelIndex MotorOnWheelIdx);

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
    void setTargetRPMs(float x, float y, float r);

    // Uses the desiredOutputs matrix to set the desired power of the motors
    void setDesiredOutput(WheelIndex WheelIdx, MotorOnWheelIndex MotorOnWheelIdx);

    void calculateMecanum(float x, float y, float r, float maxWheelSpeed);  // normal 4wd mecanum robots
    void calculateSwerve(float x, float y, float r, float maxWheelSpeed);   // swerve drive robots
    void calculateRail(float x, float maxWheelSpeed);                       // sentry rail robots

    inline int getNumChassisMotors() const override { return DRIVEN_WHEEL_COUNT * MOTORS_PER_WHEEL; }

    inline float getDesiredRotation() const { return desiredRotation; }

    void limitChassisPower();

    /**
     * @return A number between 0 and 1 that is the ratio between the rotationRpm and
     *      the max rotation speed.
     */
    mockable float calculateRotationTranslationalGain(float chassisRotationDesiredWheelspeed);

    static inline float getMaxRefWheelSpeed(bool refSerialOnline, int chassisPower) {
        if (refSerialOnline) {
            float desWheelSpeed = WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE *
                                      static_cast<float>(chassisPower - MIN_CHASSIS_POWER) +
                                  MIN_WHEEL_SPEED_SINGLE_MOTOR;

            return tap::algorithms::limitVal(
                desWheelSpeed,
                static_cast<float>(MIN_WHEEL_SPEED_SINGLE_MOTOR),
                static_cast<float>(MAX_WHEEL_SPEED_SINGLE_MOTOR));
        } else {
            // return MIN_WHEEL_SPEED_SINGLE_MOTOR;
            return MAX_WHEEL_SPEED_SINGLE_MOTOR;
        }
    }

    inline bool allMotorsOnline() const override {
        for (int i = 0; i < DRIVEN_WHEEL_COUNT; i++) {
            for (int j = 0; j < MOTORS_PER_WHEEL; j++) {
                if (!motors[i][j]->isMotorOnline()) {
                    return false;
                }
            }
        }
        return true;
    }

    Matrix<float, 3, 1> getActualVelocityChassisRelative() const override {
        // no proper override because we don't have a need for this function yet
        return Matrix<float, 3, 1>::zeroMatrix();
    }

#ifndef ENV_UNIT_TESTS
   private:
#else
   public:
#endif
    src::Drivers* drivers;
    float desiredRotation = 0.0f;

#ifdef TARGET_SENTRY
    DJIMotor railWheel;
    SmoothPID railWheelVelPID;

#else
    DJIMotor leftBackWheel, leftFrontWheel, rightFrontWheel, rightBackWheel;
    SmoothPID leftBackWheelVelPID, leftFrontWheelVelPID, rightFrontWheelVelPID, rightBackWheelVelPID;
#ifdef SWERVE
    DJIMotor leftBackYaw, leftFrontYaw, rightFrontYaw, rightBackYaw;
    SmoothPID leftBackYawPosPID, leftFrontYawPosPID, rightFrontYawPosPID, rightBackYawPosPID;
#endif
#endif

    Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> targetRPMs;
    Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> desiredOutputs;

    Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> motors;

    Matrix<SmoothPID*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> velocityPIDs;

    src::utils::Control::PowerLimiting::PowerLimiter powerLimiter;

    Matrix<float, 4, 3> wheelLocationMatrix;

   public:
#ifdef TARGET_SENTRY
    inline int16_t getLeftFrontRpmActual() const override {
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