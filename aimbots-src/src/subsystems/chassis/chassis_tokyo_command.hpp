#pragma once

#include "tap/algorithms/ramp.hpp"

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Chassis {

struct ToykoRandomizerConfig {
    float minSpinRateModifier = 0.75f;
    float maxSpinRateModifier = 1.0f;
    uint32_t minSpinRateModifierDuration = 500;
    uint32_t maxSpinRateModifierDuration = 3000;
};

class ChassisTokyoCommand : public TapCommand {
public:
    ChassisTokyoCommand(
        src::Drivers*,
        ChassisSubsystem*,
        src::Gimbal::GimbalSubsystem*,
        int spinDirectionOverride = 0,
        bool randomizeSpinRate = false,
        const ToykoRandomizerConfig& randomizerConfig = ToykoRandomizerConfig());
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    void setRotationDirection(bool rotateLeft) { rotationDirection = (rotateLeft ? 1 : -1); }
    void randomizeSpinCharacteristics();

    const char* getName() const override { return "Chassis Follow Gimbal"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;

    int spinDirectionOverride;

    float rotationDirection;
    tap::algorithms::Ramp rotationSpeedRamp;

    bool randomizeSpinRate;
    const ToykoRandomizerConfig& randomizerConfig;

    float spinRateModifier;
    uint32_t spinRateModifierDuration;
    MilliTimeout spinRateModifierTimer;
};

}  // namespace src::Chassis