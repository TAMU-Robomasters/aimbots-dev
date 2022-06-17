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

    void runYawController(AngleUnit unit, float targetChassisRelativeYawAngle) override;
    void runPitchController(AngleUnit unit, float targetChassisRelativePitchAngle) override;

    inline SmoothPID* getYawPositionPID() { return &yawPositionPID; }
    inline SmoothPID* getPitchPositionPID() { return &pitchPositionPID; }

    bool isOnline() const;

   private:
    GimbalSubsystem* gimbal;

    SmoothPID yawPositionPID;
    SmoothPID pitchPositionPID;
};

}  // namespace src::Gimbal