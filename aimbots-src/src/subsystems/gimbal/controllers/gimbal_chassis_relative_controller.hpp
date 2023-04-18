#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_controller_interface.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <utils/common_types.hpp>
#include <utils/pid/smooth_pid_wrap.hpp>

namespace src::Gimbal {

class GimbalChassisRelativeController : public GimbalControllerInterface {
public:
    GimbalChassisRelativeController(GimbalSubsystem*);

    void initialize() override;

    void BuildPositionPIDs() {
        for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
            yawPositionPIDs[i] = new SmoothPID(YAW_POSITION_PID_CONFIG);
        }
        for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
            pitchPositionPIDs[i] = new SmoothPID(PITCH_POSITION_PID_CONFIG);
        }
    }

    void runYawController(AngleUnit unit, float targetChassisRelativeYawAngle, bool vision = false) override;
    void runPitchController(AngleUnit unit, float targetChassisRelativePitchAngle, bool vision = false) override;

    inline SmoothPID* getYawPositionPID() { return yawPositionPIDs[0]; }
    inline SmoothPID* getPitchPositionPID() { return pitchPositionPIDs[0]; }

    bool isOnline() const;

    float getTargetYaw(AngleUnit unit) const override { return gimbal->getTargetYawAngle(unit); }

private:
    GimbalSubsystem* gimbal;

    std::array<SmoothPID*, YAW_MOTOR_COUNT> yawPositionPIDs;
    std::array<SmoothPID*, PITCH_MOTOR_COUNT> pitchPositionPIDs;
};

}  // namespace src::Gimbal