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

    SmoothPID yawPositionPID;
    SmoothPID pitchPositionPID;

    SmoothPID yawVisionPID;
    SmoothPID pitchVisionPID;
};

}  // namespace src::Gimbal