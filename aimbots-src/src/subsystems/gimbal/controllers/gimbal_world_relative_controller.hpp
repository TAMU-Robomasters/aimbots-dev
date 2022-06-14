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

    int yawRevolutions = 0;

    float previousYaw = 0.0f;
    float worldSpaceYawTarget = 0.0f;
    float chassisRelativeInitialIMUYaw = 0.0f;

    float worldSpacePitchTarget = 0.0f;
    float chassisRelativeInitialIMUPitch = 0.0f;

    inline float getBMIYawUnwrapped() const {
        return drivers->fieldRelativeInformant.getYaw() + (M_TWOPI * yawRevolutions);
    }

    void updateYawRevolutionCounter() {
        float newYaw = drivers->fieldRelativeInformant.getYaw();
        float diff = newYaw - previousYaw;
        previousYaw = newYaw;

        if (diff < -M_PI) {
            yawRevolutions++;
        } else if (diff > M_PI) {
            yawRevolutions--;
        }
    }
    
    float getGimbalForwardRelativeIMUPitch() const {
        // This is the gimbal's current angle, given that the
        // chassis forward direction is the +y-axis, and its
        // right direction is the +x-axis.
        float theta = (modm::toRadian(YAW_START_ANGLE) - gimbal->getCurrentYawAngle(AngleUnit::Radians)) + M_PI_2;

        float sin_theta = sin(theta);
        float pitch = modm::toRadian(drivers->fieldRelativeInformant.getPitch());

        float cos_theta = cos(theta);
        float roll = modm::toRadian(drivers->fieldRelativeInformant.getRoll());

        return (abs(cos_theta) >= abs(sin_theta)) ? (pitch * sin_theta) : (roll * cos_theta);
    }

    SmoothPID yawPositionPID;
    SmoothPID pitchPositionPID;
};

}  // namespace src::Gimbal