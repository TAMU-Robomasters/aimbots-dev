#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_controller_interface.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <utils/common_types.hpp>
#include <utils/pid/smooth_pid_wrap.hpp>

namespace src::Gimbal {

class GimbalWorldRelativeController : public GimbalControllerInterface {
   public:
    GimbalWorldRelativeController(src::Drivers*, GimbalSubsystem*);

    void initialize() override;

    void runYawController(AngleUnit unit, float targetYawAngle) override;
    void runPitchController(AngleUnit unit, float targetPitchAngle) override;

    bool isOnline() const;

   private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;

    int chassisRevolutions = 0;

    float prevWrappedChassisAngle = 0.0f;
    float worldSpaceYawTarget = 0.0f;
    float prevUnwrappedChassisAngle = 0.0f;

    // uses IMU to get total chassis rotation since initialization
    inline float getUnwrappedChassisAngle() const {
        return drivers->fieldRelativeInformant.getYaw() + (M_TWOPI * chassisRevolutions);
    }

    void updateChassisRevolutionCounter() {
        float newWrappedChassisAngle = drivers->fieldRelativeInformant.getYaw();
        float diff = newWrappedChassisAngle - prevWrappedChassisAngle;

        if (diff < -M_PI) {
            chassisRevolutions++;
        } else if (diff > M_PI) {
            chassisRevolutions--;
        }
        prevWrappedChassisAngle = newWrappedChassisAngle;
    }

    SmoothPID yawPositionPID;
    SmoothPID pitchPositionPID;
};

}  // namespace src::Gimbal