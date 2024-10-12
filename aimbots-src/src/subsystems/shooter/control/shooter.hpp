#pragma once

#include "tap/control/subsystem.hpp"

#include "utils/tools/common_types.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {

class ShooterSubsystem : public tap::control::Subsystem {
public:
    ShooterSubsystem(tap::Drivers* drivers);

    mockable void initialize() override;
    void refresh() override;

    float getMotorRPM(uint8_t motorIdx) const;

    void updateMotorVelocityPID(uint8_t motorIdx);

    void setTargetRPM(uint8_t motorIdx, float targetRPM);

    void setDesiredOutput(uint8_t motorIdx, float desiredOutput);

    void setDesiredOutputToMotor(uint8_t motorIdx);

    bool isOnline() const { return false; }

private:
    tap::Drivers* drivers;
    
    DJIMotor flywheel1, flywheel2;
    SmoothPID flywheel1PID, flywheel2PID;

    std::array<float, SHOOTER_MOTOR_COUNT> targetRPMs;
    std::array<int32_t, SHOOTER_MOTOR_COUNT> desiredOutputs;
    std::array<DJIMotor*, SHOOTER_MOTOR_COUNT> motors;

    std::array<SmoothPID*, SHOOTER_MOTOR_COUNT> velocityPIDs;
};
};  // namespace src::Shooter

#endif  //#ifdef SHOOTER_COMPATIBLE