#pragma once

#include "drivers.hpp"
#include "subsystems/chassis/basic_commands/chassis_ignore_gimbal_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_tokyo_master_command.hpp"
#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/chassis/control/chassis_helper.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

/**
 * - chassisIgnoreGimbalCommand is scheduled for regular drive (no spin)
 * - tokyoMasterCommand is scheduled whenever F toggle, button 1, or button 2 is pressed
 * - button 2 has priority over button 1 and selects sinusodal spin mode
 * - joystick 2 X pos is passed into Tokyo Master as a manual spin override while Tokyo Master is active
 * - button 4 selects highMaxWheelSpeed as the requested ceiling while held
 * - the requested ceiling is then reduced dynamically using the power sensor
 */
class ChassisToggleDriveCustomControllerCommand : public TapComprisedCommand {
public:
    ChassisToggleDriveCustomControllerCommand(
        src::Drivers* drivers,
        ChassisSubsystem* chassis,
        Gimbal::GimbalSubsystem* gimbal,
        const TokyoConfig& tokyoConfig = TokyoConfig(),
        bool randomizeSpinRate = false,
        const SpinRandomizerConfig& randomizerConfig = SpinRandomizerConfig(),
        float normalMaxWheelSpeed = 6500.0f,
        float highMaxWheelSpeed = 10000.0f);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    char const* getName() const override { return "Chassis Toggle Drive Custom Controller Command"; }

private:
    float applyDeadband(float value, float deadband) const;
    float slewToward(float current, float target, float increaseStep, float decreaseStep) const;
    float calculatePowerLimitedMaxWheelSpeed(float requestedMaxWheelSpeed);

    void scheduleIgnoreGimbal(bool interrupted = true);
    void scheduleTokyoMaster();

    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    ChassisIgnoreGimbalCommand ignoreGimbalCommand;
    ChassisTokyoMasterCommand tokyoMasterCommand;

    float normalMaxWheelSpeed;
    float highMaxWheelSpeed;

    float powerLimitedMaxWheelSpeed = 0.0f;

    bool wasFPressed = false;
    bool tokyoMasterEnabledByF = false;
    int preferredSpinDirection = 0;

    MilliTimeout qPressed;
    MilliTimeout ePressed;

    static constexpr float MANUAL_SPIN_DEADBAND = 0.05f;

    // closed-loop chassis input ceiling
    // POWER_LIMIT_SCALAR < 1.0 cuts harder, > 1.0 cuts softer.
    static constexpr bool POWER_LIMITING_ENABLED = true;
    static constexpr uint32_t POWER_SENSOR_FRESH_TIMEOUT_MS = 50;
    static constexpr float TARGET_CHASSIS_POWER_W = 100.0f;
    static constexpr float POWER_LIMIT_SCALAR = 1.5f;
    static constexpr float POWER_LIMIT_MIN_SCALE = 0.20f;

    // rpm buffer system. POWER_LIMIT_RPM_DECREASE_PER_ITER is the amount to decrease the target rpm by whenever
    // exceeding the power limit. POWER_LIMIT_RPM_RECOVERY_PER_ITER is rate at which the rpm recovers when back in power range to manage the acceleration
    static constexpr float POWER_LIMIT_RPM_DECREASE_PER_ITER = 300.0f;
    static constexpr float POWER_LIMIT_RPM_RECOVERY_PER_ITER = 20.0f;
};

}  // namespace src::Chassis

#endif  // #ifdef CHASSIS_COMPATIBLE