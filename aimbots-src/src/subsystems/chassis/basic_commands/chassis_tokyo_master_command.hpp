#pragma once

#include <cstdint>

#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "drivers.hpp"
#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/chassis/control/chassis_helper.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"

#ifdef GIMBAL_UNTETHERED
#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

enum class ChassisTokyoMasterMode : uint8_t {
    NORMAL = 0,
    SINUSODAL = 1,
};

class ChassisTokyoMasterCommand : public TapCommand {
public:
    ChassisTokyoMasterCommand(
        src::Drivers* drivers,
        ChassisSubsystem* chassis,
        src::Gimbal::GimbalSubsystem* gimbal,
        const TokyoConfig& tokyoConfig = TokyoConfig(),
        int spinDirectionOverride = 0,
        bool randomizeSpinRate = false,
        const SpinRandomizerConfig& randomizerConfig = SpinRandomizerConfig(),
        ChassisTokyoMasterMode mode = ChassisTokyoMasterMode::NORMAL,
        float joystick2OverrideVelocity = 0.0f,
        float maxWheelSpeed = 4000.0f);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Chassis Tokyo Master"; }

    void configure(
        ChassisTokyoMasterMode mode,
        float joystick2OverrideVelocity,
        float maxWheelSpeed,
        int spinDirectionOverride = 0);

    void setMode(ChassisTokyoMasterMode mode) { this->mode = mode; }
    void setJoystick2OverrideVelocity(float joystick2OverrideVelocity);
    void setMaxWheelSpeed(float maxWheelSpeed);
    void setSpinDirectionOverride(int spinDirectionOverride);

private:
    int pickRandomDirection() const;
    int sanitizeSpinDirection(int spinDirection) const;
    void refreshActiveSpinDirectionIfNeeded();

    float applyDeadband(float value, float deadband) const;
    float getBaseAutoSpinTarget(float maxWheelSpeed) const;
    float getRandomizedNormalSpinTarget(float maxWheelSpeed);
    float getSinusodalSpinTarget(float maxWheelSpeed);

    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;

    TokyoConfig tokyoConfig;
    int spinDirectionOverride = 0;
    bool randomizeSpinRate = false;
    SpinRandomizerConfig randomizerConfig;

    ChassisTokyoMasterMode mode = ChassisTokyoMasterMode::NORMAL;
    float joystick2OverrideVelocity = 0.0f;
    float maxWheelSpeed = 4000.0f;

    tap::algorithms::Ramp rotationSpeedRamp;
    tap::arch::MilliTimeout spinRateModifierTimer;

    int activeSpinDirection = 1;
    int lastSpinDirectionOverride = 0;
    ChassisTokyoMasterMode lastMode = ChassisTokyoMasterMode::NORMAL;

    float spinRateModifier = 1.0f;
    uint32_t spinRateModifierDuration = 0;

    static constexpr float JOYSTICK_OVERRIDE_DEADBAND = 0.05f;
    static constexpr float TRANSLATION_DEADBAND = 0.03f;
};

}  // namespace src::Chassis

#endif  // #ifdef CHASSIS_COMPATIBLE
#endif  // #ifdef GIMBAL_UNTETHERED
