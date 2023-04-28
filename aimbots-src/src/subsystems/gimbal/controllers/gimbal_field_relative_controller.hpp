#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_controller_interface.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <utils/common_types.hpp>
#include <utils/pid/smooth_pid_wrap.hpp>

namespace src::Gimbal {

class GimbalFieldRelativeController : public GimbalControllerInterface {
public:
    GimbalFieldRelativeController(src::Drivers*, GimbalSubsystem*);

    void initialize() override;

    void BuildPositionPIDs() {
        for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
            yawPositionPIDs[i] = new SmoothPID(YAW_POSITION_PID_CONFIG);
            yawVisionPositionPIDs[i] = new SmoothPID(YAW_VISION_PID_CONFIG);
        }
        for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
            pitchPositionPIDs[i] = new SmoothPID(PITCH_POSITION_PID_CONFIG);
            pitchVisionPositionPIDs[i] = new SmoothPID(PITCH_VISION_PID_CONFIG);
        }
    }

    void runYawController(AngleUnit unit, float desiredFieldRelativeYawAngle, bool vision = false) override;
    void runPitchController(AngleUnit unit, float desiredChassisRelativePitchAngle, bool vision = false) override;

    bool isOnline() const;

    float getTargetYaw(AngleUnit unit) const override {
        return (unit == AngleUnit::Degrees) ? fieldRelativeYawTarget : modm::toDegree(fieldRelativeYawTarget);
    }

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;

    float fieldRelativeYawTarget = 0.0f;

    std::array<SmoothPID*, YAW_MOTOR_COUNT> yawPositionPIDs;
    std::array<SmoothPID*, PITCH_MOTOR_COUNT> pitchPositionPIDs;

    std::array<SmoothPID*, YAW_MOTOR_COUNT> yawVisionPositionPIDs;
    std::array<SmoothPID*, PITCH_MOTOR_COUNT> pitchVisionPositionPIDs;
};

}  // namespace src::Gimbal