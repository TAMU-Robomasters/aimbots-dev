#pragma once

#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "utils/common_types.hpp"
#include "utils/motion/power_limiter/power_limiter.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

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

/**
 * Used to index into matrices returned by functions of the form get*Velocity*().
 */
enum ChassisVelIndex {
    X = 0,
    Y = 1,
    R = 2,
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
            (motors[i][DRIVER]->*func)(args...);
#ifdef SWERVE
            (motors[i][YAW]->*func)(args...);
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

    void calculateHolonomic(float x, float y, float r, float maxWheelSpeed);  // normal 4wd mecanum robots
#ifdef SWERVE
    void calculateSwerve(float x, float y, float r, float maxWheelSpeed);  // swerve drive robots
#endif
    void calculateRail(float x, float maxWheelSpeed);  // sentry rail robots

    inline int getNumChassisMotors() const override { return DRIVEN_WHEEL_COUNT * MOTORS_PER_WHEEL; }

    inline float getDesiredRotation() const { return desiredRotation; }

    void limitChassisPower();

    /**
     * @return A number between 0 and 1 that is the ratio between the rotationRpm and
     *      the max rotation speed.
     */
    mockable float calculateRotationLimitedTranslationalWheelspeed(
        float chassisRotationDesiredWheelspeed,
        float maxWheelSpeed);

    /**
     * @return Returns the maximum wheel speed that can be reasonably achieved while maintaining the current power limit.
     */
    static inline float getMaxRefWheelSpeed(bool refSerialOnline, int chassisPowerLimit) {
        if (refSerialOnline) {
            float desWheelSpeed =
                WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE * static_cast<float>(chassisPowerLimit - MIN_CHASSIS_POWER) +
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

    /**
     * Converts the velocity matrix from raw RPM to wheel velocity in m/s.
     */
    inline modm::Matrix<float, 4, 1> convertRawRPM(const modm::Matrix<float, 4, 1>& mat) const {
        static constexpr float ratio = 2.0f * M_PI * CHASSIS_GEARBOX_RATIO / 60.0f;
        return mat * ratio;
    }

    Matrix<float, 3, 1> getActualVelocityChassisRelative() const override {
        Matrix<float, DRIVEN_WHEEL_COUNT, 1> wheelVelocities;

        wheelVelocities[LF][0] = leftFrontWheel.getShaftRPM();
        wheelVelocities[RF][0] = rightFrontWheel.getShaftRPM();
        wheelVelocities[LB][0] = leftBackWheel.getShaftRPM();
        wheelVelocities[RB][0] = rightBackWheel.getShaftRPM();

        return wheelVelToChassisVelMat * convertRawRPM(wheelVelocities);
    }

    bool getTokyoDrift() const;
    void setTokyoDrift(bool drift) { tokyoDrift = drift; }

#ifndef ENV_UNIT_TESTS
private:
#else
public:
#endif
    src::Drivers* drivers;
    float desiredRotation = 0.0f;

    DJIMotor leftBackWheel, leftFrontWheel, rightFrontWheel, rightBackWheel;
    SmoothPID leftBackWheelVelPID, leftFrontWheelVelPID, rightFrontWheelVelPID, rightBackWheelVelPID;
#ifdef SWERVE
    DJIMotor leftBackYaw, leftFrontYaw, rightFrontYaw, rightBackYaw;
    SmoothPID leftBackYawPosPID, leftFrontYawPosPID, rightFrontYawPosPID, rightBackYawPosPID;
#endif
    Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> targetRPMs;
    Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> desiredOutputs;

    Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> motors;

    Matrix<SmoothPID*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL> velocityPIDs;

    src::Utils::Control::PowerLimiting::PowerLimiter powerLimiter;

    bool tokyoDrift;

protected:
    Matrix<float, 3, 4> wheelVelToChassisVelMat;

public:
    inline int16_t getLeftFrontRpmActual() const override { return leftFrontWheel.getShaftRPM(); }
    inline int16_t getLeftBackRpmActual() const override { return leftBackWheel.getShaftRPM(); }
    inline int16_t getRightFrontRpmActual() const override { return rightFrontWheel.getShaftRPM(); }
    inline int16_t getRightBackRpmActual() const override { return rightBackWheel.getShaftRPM(); }
};

};  // namespace src::Chassis