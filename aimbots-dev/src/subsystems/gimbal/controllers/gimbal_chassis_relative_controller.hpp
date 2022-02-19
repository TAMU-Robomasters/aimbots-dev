#pragma once

#include <drivers.hpp>
#include <utils/common_types.hpp>
#include <subsystems/gimbal/gimbal.hpp>

namespace src::Gimbal {

class GimbalChassisRelativeController {
public:
    GimbalChassisRelativeController(GimbalSubsystem*,
                                    tap::algorithms::SmoothPidConfig const& yawPIDConfig,
                                    tap::algorithms::SmoothPidConfig const& pitchPIDConfig);

    void initialize();

    void runYawController(float deltaTime, float targetYawAngle);
    void runPitchController(float deltaTime, float targetPitchAngle);

    bool isOnline() const;
private:
    GimbalSubsystem* gimbal;

    SmoothPID yawPID;
    SmoothPID pitchPID;
};

}  // src::Gimbal