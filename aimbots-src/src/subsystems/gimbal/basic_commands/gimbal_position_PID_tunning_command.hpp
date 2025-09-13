#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/control/gimbal.hpp>
#include <tap/control/command.hpp>

#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"

#ifdef GIMBAL_COMPATIBLE
namespace src::Gimbal {

struct GimbalPositionTunningConfig {
    float positionAmplitudeDegrees = 0.0f;
    float frequencyHz = 0.0f;
};

class GimbalPositionTunningCommand : public tap::control::Command {
public:
    GimbalPositionTunningCommand(
        src::Drivers* drivers,
        GimbalSubsystem* gimbalSubsystem,
        GimbalFieldRelativeController* controller,
        GimbalPositionTunningConfig yawConfig,
        GimbalPositionTunningConfig pitchConfig);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;
    bool isReady() override;

    const char* getName() const { return "gimbal position tunning"; }

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;
    GimbalFieldRelativeController* controller;
    GimbalPositionTunningConfig yawConfig;
    GimbalPositionTunningConfig pitchConfig;
    uint32_t initTime;

    float getYawTargetPosition();
    float getPitchTargetPosition();
    float getRelativeTime() const;
};

} // namespace src::Gimbal 
#endif // #ifdef GIMBAL_COMPATIBLE