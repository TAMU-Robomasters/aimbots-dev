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

    void runYawController(AngleUnit unit, float desiredFieldSpaceYawAngle) override;
    void runPitchController(AngleUnit unit, float desiredChassisSpacePitchAngle) override;

    bool isOnline() const;

    inline float getFieldSpaceTargetYaw(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? fieldSpaceYawTarget : modm::toDegree(fieldSpaceYawTarget); }

   private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;

    float fieldSpaceYawTarget = 0.0f;

    SmoothPID yawPositionPID;
    SmoothPID pitchPositionPID;
};

}  // namespace src::Gimbal