#pragma once
#ifndef ENGINEER
#include "drivers.hpp"
#include "informants/limit_switch.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Shooter {
class ShooterSubsystem : public tap::control::Subsystem {
   public:
    FeederSubsystem(
        src::Drivers* drivers);

    mockable void initialize() override;
    // code below altered to follow shooter.hpp:56
    void refresh() override;

    void updateMotorVelocityPID(MotorIndex motorIdx);

    /**
     * @brief Changes the target RPM for a single motor. Intended for use with ForAllShooterMotors(),
     * and should be called from a command to declare intended RPM. Does not necessarily need to be called continuously
     * 
     * @param motorIdx index for DJIMotor matrix
     * @param targetRPM intended target RPM
     */
    void setTargetRPM(MotorIndex MotorInde, float targetRPM);

    /**
     * @brief Updates the desiredOutputs matrix with the desired output of a single motor.
     * Intended to be called from commands.
     *
     * @param motorIdx
     * @param desiredOutput
     */
    void setDesiredOutput(MotorIndex motorIdx, float desiredOutput);

    float getTargetRPM() const {
        return targetRPM;
    }

    float getCurrentRPM() const {
        return shooterMotor.getShaftRPM();
    }

    int getTotalLimitCount() const;

    bool isBarrelHeatAcceptable(float maxPercentage);

#ifndef ENV_UNIT_TESTS
   private:
#else
   public:
#endif

    float targetRPM;
    float desiredOutput;

    SmoothPID shooterVelPID;
    DJIMotor shooterMotor;

    src::Informants::LimitSwitch limitSwitchLeft;
};

}

#endif