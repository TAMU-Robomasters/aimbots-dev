#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/control/gimbal.hpp>
#include <tap/control/command.hpp>

#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"

#ifdef GIMBAL_COMPATIBLE
namespace src::Gimbal {

struct GimbalTestConfig {
    float pitchAmplitudeDegree;
    float pitchFrequencyHz;
    float pitchOffsetDegree;
    float yawAmplitudeDegree;
    float yawFrequencyHz;
    float yawOffsetDegree;
};

static constexpr int_fast8_t NUM_TRANSFORM_ELEMENTS = 4*4;

class GimbalTestCommand : public tap::control::Command {
public: 
    GimbalTestCommand(src::Drivers*, GimbalSubsystem*, GimbalFieldRelativeController*, GimbalTestConfig);
    
    char const* getName() const override { return "Gimbal Test Command"; }

    void initialize() override;
    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

    float getYawTargetAngle();
    float getPitchTargetAngle();
    
    uint32_t getRelativeTime() {return tap::arch::clock::getTimeMilliseconds() - initTime;}
    void resetInitTime() {initTime = tap::arch::clock::getTimeMilliseconds();}

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;
    GimbalFieldRelativeController* controller;
    GimbalTestConfig config;

    bool wasJetsonOnline = false;
    uint32_t initTime = 0;
};

} // namespace src::Gimbal
#endif // #ifdef GIMBAL_COMPATIBLE
