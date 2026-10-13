#pragma once

#include "tap/algorithms/ramp.hpp"

#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/chassis/control/chassis_helper.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

#if defined(CHASSIS_COMPATIBLE) && defined(GIMBAL_COMPATIBLE)

namespace src::Chassis {

/**
 * "Spin-to-win" driven by nav2. Same continuous-rotation tokyo drift as ChassisTokyoCommand, but the
 * translation input comes from the Jetson's field-relative velocity command (m/s) instead of the
 * operator. The velocity is scaled into the chassis input range by a tunable global multiplier, then
 * fed through the identical tokyo translation/rotation pipeline (spin ramp, randomizer, rotation
 * slowdown while translating) and rotated into the chassis frame in two steps: field-relative ->
 * turret-relative using the turret's field-relative yaw (from its IMU), then turret-relative ->
 * chassis-relative using the gimbal's yaw-from-center.
 */
class ChassisAutoNavTokyoVelocityCommand : public TapCommand {
public:
    ChassisAutoNavTokyoVelocityCommand(
        src::Drivers*,
        ChassisSubsystem*,
        src::Gimbal::GimbalSubsystem*,
        const TokyoConfig& tokyoConfig,
        int spinDirectionOverride = 0,
        bool randomizeSpinRate = false,
        const SpinRandomizerConfig& randomizerConfig = SpinRandomizerConfig());

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    void setRotationDirection(bool rotateLeft) { rotationDirection = (rotateLeft ? 1 : -1); }

    const char* getName() const override { return "Chassis Auto Nav Tokyo Velocity"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;

    const TokyoConfig& tokyoConfig;

    int spinDirectionOverride;

    float rotationDirection;
    tap::algorithms::Ramp rotationSpeedRamp;

    bool randomizeSpinRate;
    const SpinRandomizerConfig& randomizerConfig;

    float spinRateModifier;
    uint32_t spinRateModifierDuration;
    MilliTimeout spinRateModifierTimer;
};

}  // namespace src::Chassis

#endif  // defined(CHASSIS_COMPATIBLE) && defined(GIMBAL_COMPATIBLE)
