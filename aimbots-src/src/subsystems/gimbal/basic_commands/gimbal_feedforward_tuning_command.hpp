#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/control/gimbal.hpp>
#include <tap/control/command.hpp>

#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"

#include "subsystems/gimbal/control/gimbal_helper.hpp"

#ifdef GIMBAL_COMPATIBLE
namespace src::Gimbal {



class GimbalFeedforwardTunningCommand : public tap::control::Command {
public:
    GimbalFeedforwardTunningCommand(
        src::Drivers* drivers,
        GimbalSubsystem* gimbalSubsystem,
        GimbalFieldRelativeController* controller,
        GimbalFeedForwardConfig& ffconfig);

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
    GimbalFeedForwardConfig ffConfig
    uint32_t initTime;
    Vector<float> pitchTargets;

    float getYawTargetPosition();
    vector<float> getPitchTargets();
    float getRelativeTime() const;
};

} // namespace src::Gimbal 
#endif // #ifdef GIMBAL_COMPATIBLE