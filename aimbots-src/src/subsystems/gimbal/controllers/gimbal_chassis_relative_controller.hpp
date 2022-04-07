#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <utils/common_types.hpp>
#include <utils/pid/smooth_pid_wrap.hpp>

namespace src::Gimbal {

class GimbalChassisRelativeController {
   public:
    GimbalChassisRelativeController(GimbalSubsystem*);

    void initialize();

    void runYawController(AngleUnit unit, float targetYawAngle);
    void runPitchController(AngleUnit unit, float targetPitchAngle);

    bool isOnline() const;

   private:
    GimbalSubsystem* gimbal;

    SmoothPID yawPositionPID;
    SmoothPID pitchPositionPID;
};

}  // namespace src::Gimbal