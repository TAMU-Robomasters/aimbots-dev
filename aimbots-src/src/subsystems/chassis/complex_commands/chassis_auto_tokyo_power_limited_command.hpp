#pragma once

#include "drivers.hpp"
#include "subsystems/chassis/basic_commands/chassis_tokyo_master_command.hpp"
#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/chassis/control/chassis_helper.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

/**
 * Autonomous (sentry) copy of ChassisToggleDriveCustomControllerCommand:
 * - Tokyo Master is ALWAYS scheduled ("spin to win") — no F toggle, no custom controller buttons
 * - Tokyo Master runs with isAuto=true, so translation comes from the Jetson's nav2 velocity
 *   command instead of the operator, and all operator/custom-controller input is ignored
 * - the requested wheel-speed ceiling is reduced dynamically using the ESP power sensor,
 *   identical logic to the custom controller command
 */
class ChassisAutoTokyoPowerLimitedCommand : public TapComprisedCommand {
public:
    ChassisAutoTokyoPowerLimitedCommand(
        src::Drivers* drivers,
        ChassisSubsystem* chassis,
        Gimbal::GimbalSubsystem* gimbal,
        const TokyoConfig& tokyoConfig = TokyoConfig(),
        bool randomizeSpinRate = false,
        const SpinRandomizerConfig& randomizerConfig = SpinRandomizerConfig(),
        float maxWheelSpeed = 5000.0f);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    char const* getName() const override { return "Chassis Auto Tokyo Power Limited Command"; }

private:
    float slewToward(float current, float target, float increaseStep, float decreaseStep) const;
    float calculatePowerLimitedMaxWheelSpeed(float requestedMaxWheelSpeed);

    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    ChassisTokyoMasterCommand tokyoMasterCommand;

    float maxWheelSpeed;

    float powerLimitedMaxWheelSpeed = 0.0f;

    // closed-loop chassis input ceiling
    // POWER_LIMIT_SCALAR < 1.0 cuts harder, > 1.0 cuts softer.
    static constexpr bool POWER_LIMITING_ENABLED = true;
    static constexpr uint32_t POWER_SENSOR_FRESH_TIMEOUT_MS = 50;
    static constexpr float TARGET_CHASSIS_POWER_W = 75.0f;
    static constexpr float POWER_LIMIT_SCALAR = 1.5f;
    static constexpr float POWER_LIMIT_MIN_SCALE = 0.20f;

    // rpm buffer system. POWER_LIMIT_RPM_DECREASE_PER_ITER is the amount to decrease the target rpm by whenever
    // exceeding the power limit. POWER_LIMIT_RPM_RECOVERY_PER_ITER is rate at which the rpm recovers when back in power range to manage the acceleration
    static constexpr float POWER_LIMIT_RPM_DECREASE_PER_ITER = 300.0f;
    static constexpr float POWER_LIMIT_RPM_RECOVERY_PER_ITER = 20.0f;
};

}  // namespace src::Chassis

#endif  // #ifdef CHASSIS_COMPATIBLE
