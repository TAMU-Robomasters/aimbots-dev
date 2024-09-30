#pragma once

#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/motion/auto_nav/auto_navigator_holonomic.hpp"

#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/chassis/control/chassis_helper.hpp"
#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

class ChassisAutoNavTokyoCommand : public TapCommand {
public:
    ChassisAutoNavTokyoCommand(
        src::Drivers* drivers,
        ChassisSubsystem* chassis,
        SmoothPIDConfig linearPIDConfig,
        const TokyoConfig& tokyoConfig = TokyoConfig(),
        bool randomizeSpinRate = false,
        const SpinRandomizerConfig& randomizerConfig = SpinRandomizerConfig(),
        float linearSettledThreshold = 0.05f);
    ~ChassisAutoNavTokyoCommand() = default;

    void initialize() override;
    void execute() override;

    void setTargetLocation(double x, double y) {
        UNUSED(x);
        UNUSED(y);
        // autoNavigator.setTargetLocation(targetLocation);
    }

    bool isSettled() {
        return xController.isSettled(linearSettledThreshold, 0) && yController.isSettled(linearSettledThreshold, 0);
    }

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

    void randomizeSpinCharacteristics();

    const char* getName() const override { return "Chassis Auto Nav"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    AutoNav::AutoNavigatorHolonomic autoNavigator;

    SmoothPID xController;
    SmoothPID yController;

    const TokyoConfig& tokyoConfig;

    uint8_t rotationDirection;

    tap::algorithms::Ramp xRamp;
    tap::algorithms::Ramp yRamp;
    tap::algorithms::Ramp rotationSpeedRamp;

    bool randomizeSpinRate;
    const SpinRandomizerConfig& randomizerConfig;

    float spinRateModifier;
    uint32_t spinRateModifierDuration;
    MilliTimeout spinRateModifierTimer;

    float linearVelocityRampValue = 1.0f;

    float linearSettledThreshold;
};

}  // namespace src::Chassis

#endif  //#ifdef CHASSIS_COMPATIBLE