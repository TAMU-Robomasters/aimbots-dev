#pragma once
#include <vector>

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Shooter {

enum MotorIndex {
    RIGHT = 0,
    LEFT = 1,
#ifdef TARGET_SENTRY
    TOP_RIGHT = 0,
    BOT_RIGHT = 1,
    TOP_LEFT = 2,
    BOT_LEFT = 3,
#endif
};

class ShooterSubsystem : public tap::control::Subsystem {
public:
    ShooterSubsystem(tap::Drivers* drivers, src::Utils::RefereeHelperTurreted* refHelper);

    /**
     * Allows user to call a DJIMotor member function on all shooter motors
     *
     * @param function pointer to a member function of DJIMotor
     * @param args arguments to pass to the member function
     */
    template <class... Args>
    void ForAllShooterMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto i = 0; i < SHOOTER_MOTOR_COUNT; i++) {
            (motors[i][0]->*func)(args...);
        }
    }

    /**
     * Allows user to call a ShooterSubsystem function on all shooter motors.
     *
     * @param function pointer to a member function of ShooterSubsystem that takes a MotorIndex as it's first argument
     * @param args arguments to pass to the member function
     */
    template <class... Args>
    void ForAllShooterMotors(void (ShooterSubsystem::*func)(MotorIndex, Args...), Args... args) {
        for (auto i = 0; i < SHOOTER_MOTOR_COUNT; i++) {
            MotorIndex mi = static_cast<MotorIndex>(i);
            (this->*func)(mi, args...);
        }
    }

    mockable void initialize() override;
    void refresh() override;

    float getHighestMotorSpeed() const;

    float getMotorSpeed(MotorIndex motorIdx) const;

    /**
     * @brief Updates velocity PID and motor RPM for a single motor. Can be used with ForAllShooterMotors().
     * Should be called continuously in subsystem refresh.
     *
     * @param motorIdx index for DJIMotor matrix
     */
    void updateMotorVelocityPID(MotorIndex motorIdx);

    /**
     * @brief Changes the target RPM for a single motor. Intended for use with ForAllShooterMotors(),
     * and should be called from a command to declare intended RPM. Does not necessarily need to be called continuously
     *
     * @param motorIdx index for DJIMotor matrix
     * @param targetRPM intended target RPM
     */
    void setTargetRPM(MotorIndex motorIdx, float targetRPM);

    /**
     * @brief Updates the desiredOutputs matrix with the desired output of a single motor.
     * Intended to be called from commands.
     *
     * @param motorIdx
     * @param desiredOutput
     */
    void setDesiredOutput(MotorIndex motorIdx, float desiredOutput);

    /**
     * @brief Sets the desired output of a single motor from the desiredOutputs matrix
     * Should only be called once per loop for consistency in Shooter refresh.
     *
     * @param motorIdx
     */
    void setDesiredOutputToMotor(MotorIndex motorIdx);

    bool isOnline() const {
        for (auto i = 0; i < SHOOTER_MOTOR_COUNT; i++) {
            if (!motors[i][0]->isMotorOnline()) {
                return false;
            }
        }
        return true;
    }

#ifndef ENV_UNIT_TESTS
private:
#else
public:
#endif

    DJIMotor flywheel1, flywheel2;
    SmoothPID flywheel1PID, flywheel2PID;

#ifdef TARGET_SENTRY
    DJIMotor flywheel3, flywheel4;
    SmoothPID flywheel3PID, flywheel4PID;
#endif

    Matrix<float, SHOOTER_MOTOR_COUNT, 1> targetRPMs;
    Matrix<int32_t, SHOOTER_MOTOR_COUNT, 1> desiredOutputs;
    Matrix<DJIMotor*, SHOOTER_MOTOR_COUNT, 1> motors;

    Matrix<SmoothPID*, SHOOTER_MOTOR_COUNT, 1> velocityPIDs;

    src::Utils::RefereeHelperTurreted* refHelper;
};
};  // namespace src::Shooter