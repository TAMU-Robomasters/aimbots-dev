#pragma once
  
#include <drivers.hpp>
#include <subsystems/gimbal/control/gimbal.hpp>
#include <tap/control/command.hpp>

#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"

namespace src::Gimbal {

struct GimbalTestConfig {
    float pitchAmplitude;
    float yawAmplitude;
};

class GimbalTestCommand : public tap::control::Command {
public: 
    GimbalTestCommand(src::Drivers*, GimbalSubsystem*, GimbalFieldRelativeController*, GimbalTestConfig);
    
    void initialize() override;
    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;
    GimbalFieldRelativeController* controller;
    GimbalTestConfig config;

    uint32_t commandStartTime = 0;
};

} // namespace src::Gimbal