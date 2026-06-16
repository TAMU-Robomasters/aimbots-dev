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
 * Custom controller main command scheduler command (command).
 *
 * - chassisIgnoreGimbalCommand is scheduled for non-spinning drive
 * - tokyoMasterCommand is scheduled whenever F toggle, button 1, or button 2 requests spin.
 * - button 2 has priority over button 1 and selects sinusodal spin.
 * - joystick 2 X pos is passed into Tokyo Master as a manual spin override while Tokyo Master is active.
 * - button 4 selects highMaxWheelSpeed while held (disregards power limit).
 * - button 4 mode should eventually control supercap discharge.
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
        float normalMaxWheelSpeed = 4000.0f,
        float highMaxWheelSpeed = 6500.0f);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    char const* getName() const override { return "Chassis Toggle Drive Custom Controller Command"; }

private:
    float applyDeadband(float value, float deadband) const;
    void scheduleIgnoreGimbal(bool interrupted = true);
    void scheduleTokyoMaster();

    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    ChassisIgnoreGimbalCommand ignoreGimbalCommand;
    ChassisTokyoMasterCommand tokyoMasterCommand;

    float normalMaxWheelSpeed;
    float highMaxWheelSpeed;

    bool wasFPressed = false;
    bool tokyoMasterEnabledByF = false;
    int preferredSpinDirection = 0;

    MilliTimeout qPressed;
    MilliTimeout ePressed;

    static constexpr float MANUAL_SPIN_DEADBAND = 0.05f;
};

}  // namespace src::Chassis

#endif  // #ifdef CHASSIS_COMPATIBLE
