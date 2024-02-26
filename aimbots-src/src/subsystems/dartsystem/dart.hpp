#pragma once
#include <vector>

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

#ifdef DART_COMPATIBLE

namespace src::Dart {

enum L_MotorIndex {
    L_1 = 0,
    L_2 = 1,
    L_3 = 2,
    L_4 = 3,
    L_5 = 4,
    L_6 = 5,
};

class DartSubsystem : public tap::control::Subsystem {
public:
    DartSubsystem(tap::Drivers* drivers);

    template <class... Args>
    void ForAllDartMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto i = 0; i < LAUNCH_MOTOR_COUNT; i++) {
            (motors[i][0]->*func)(args...);
        }
    }

    template <class... Args>
    void ForAllDartMotors(void (DartSubsystem::*func)(L_MotorIndex, Args...), Args... args) {
        for (auto i = 0; i < LAUNCH_MOTOR_COUNT; i++) {
            L_MotorIndex mi = static_cast<L_MotorIndex>(i);
            (this->*func)(mi, args...);
        }
    }

    mockable void initialize() override;
    void refresh() override;

    float getHighestMotorSpeed() const;

    float getMotorSpeed(L_MotorIndex motorIdx) const;

    /**
     * @brief Updates velocity PID and motor RPM for a single motor. Can be used with ForAllShooterMotors().
     * Should be called continuously in subsystem refresh.
     *
     * @param motorIdx index for DJIMotor matrix
     */
    void updateMotorVelocityPID(L_MotorIndex motorIdx);

    /**
     * @brief Changes the target RPM for a single motor. Intended for use with ForAllShooterMotors(),
     * and should be called from a command to declare intended RPM. Does not necessarily need to be called continuously
     *
     * @param motorIdx index for DJIMotor matrix
     * @param targetRPM intended target RPM
     */
    void setTargetRPM(L_MotorIndex motorIdx, float targetRPM);

    /**
     * @brief Updates the desiredOutputs matrix with the desired output of a single motor.
     * Intended to be called from commands.
     *
     * @param motorIdx
     * @param desiredOutput
     */
    void setDesiredOutput(L_MotorIndex motorIdx, float desiredOutput);

    /**
     * @brief Sets the desired output of a single motor from the desiredOutputs matrix
     * Should only be called once per loop for consistency in Shooter refresh.
     *
     * @param motorIdx
     */
    void setDesiredOutputToMotor(L_MotorIndex motorIdx);

    bool isOnline() const {
        for (auto i = 0; i < LAUNCH_MOTOR_COUNT; i++) {
            if (!motors[i][0]->isMotorOnline()) {
                return false;
            }
        }
        return true;
    }

    int isOnlineDisplay();

    

#ifndef ENV_UNIT_TESTS
private:
#else
public:
#endif

    DJIMotor launch1, launch2, launch3, launch4, launch5, launch6;
    SmoothPID launch1PID, launch2PID, launch3PID, launch4PID, launch5PID, launch6PID;


    Matrix<float, LAUNCH_MOTOR_COUNT, 1> launchTargetRPMs;
    Matrix<int32_t, LAUNCH_MOTOR_COUNT, 1> launchDesiredOutputs;
    Matrix<DJIMotor*, LAUNCH_MOTOR_COUNT, 1> motors;

    Matrix<SmoothPID*, LAUNCH_MOTOR_COUNT, 1> launchVelocityPIDs;

};
};  // namespace src::Shooter

#endif //#ifdef SHOOTER_COMPATIBLE