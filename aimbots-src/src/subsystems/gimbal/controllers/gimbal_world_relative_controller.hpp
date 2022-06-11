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

    int revolutions = 0;

    float previousYaw = 0.0f;
    float worldRelativeYaw = 0.0f;
    float chassisRelativeInitialIMUAngle = 0.0f;

    inline float getBMIYawUnwrapped() const {
        return modm::toRadian(drivers->bmi088.getYaw()) + (M_TWOPI * revolutions);
    }

    void updateRevolutionCounter() {
        float newYaw = modm::toRadian(drivers->bmi088.getYaw());
        float diff = newYaw - previousYaw;
        previousYaw = newYaw;

        if (diff < -M_PI) {
            revolutions++;
        } else if (diff > M_PI) {
            revolutions--;
        }
    }

    SmoothPID yawPositionPID;
    SmoothPID pitchPositionPID;
};

}  // namespace src::Gimbal