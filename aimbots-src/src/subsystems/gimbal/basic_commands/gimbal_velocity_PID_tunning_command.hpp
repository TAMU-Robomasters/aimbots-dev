#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/control/gimbal.hpp>
#include <tap/control/command.hpp>

#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"
#include "utils/filters/ema.hpp"

#ifdef GIMBAL_COMPATIBLE
namespace src::Gimbal {

struct GimbalVelocityTunningConfig {
    float velocityAmplitudeDegreesPerSec = 0.0f;
    float frequencyHz = 0.0f;
};  

class GimbalVelocityTunningCommand : public tap::control::Command {
public: 
    GimbalVelocityTunningCommand(src::Drivers*, GimbalSubsystem*, GimbalFieldRelativeController*, GimbalVelocityTunningConfig, GimbalVelocityTunningConfig);
    
    char const* getName() const override { return "Gimbal Velocity Test Command"; }

    void initialize() override;
    void execute() override;
    

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;
    GimbalFieldRelativeController* controller;
    GimbalVelocityTunningConfig yawConfig;
    GimbalVelocityTunningConfig pitchConfig;

    src::Utils::Filters::EMAFilter* yawVelocityFilter;

    uint32_t initTime = 0;

    float averageRPM;

    float getYawTargetVelocity();
    float getPitchTargetVelocity();
    void runYawVelocityStepOscillation(float periodSeconds);
    void runPitchVelocityStepOscillation(float periodSeconds);
    void runPitchVelocityStepUp(float periodSeconds);
    
    uint32_t getRelativeTime() {return tap::arch::clock::getTimeMilliseconds() - initTime;}
};

} // namespace src::Gimbal
#endif // #ifdef GIMBAL_COMPATIBLE