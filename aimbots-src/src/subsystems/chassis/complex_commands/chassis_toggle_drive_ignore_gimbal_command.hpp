#pragma once

#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/chassis/basic_commands/chassis_ignore_gimbal_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_tokyo_command.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"

#include "subsystems/chassis/control/chassis_helper.hpp"
#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

class ChassisToggleDriveIgnoreGimbalCommand : public TapComprisedCommand {
public:
    ChassisToggleDriveIgnoreGimbalCommand(
        src::Drivers*,
        ChassisSubsystem*,
        Gimbal::GimbalSubsystem*,
        const TokyoConfig& tokyoConfig = TokyoConfig(),
        bool randomizeSpinRate = false,
        const SpinRandomizerConfig& randomizerConfig = SpinRandomizerConfig());

    void initialize() override;
    void execute() override;

    void end(bool interupted) override;
    bool isReady() override;
    bool isFinished() const override;

    char const* getName() const override { return "Chassis Toggle Drive Ignore Gimbal Command"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    ChassisIgnoreGimbalCommand ignoreGimbalCommand;
    ChassisTokyoCommand tokyoCommand;
    ChassisTokyoCommand tokyoLeftCommand;
    ChassisTokyoCommand tokyoRightCommand;
    bool wasFPressed = false;

    bool preferSpecificSpin = false;

    MilliTimeout qPressed;
    MilliTimeout ePressed;
};

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE