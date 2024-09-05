#pragma once

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_follow_gimbal_command.hpp"
#include "subsystems/chassis/chassis_tokyo_command.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"

#include "chassis_helper.hpp"
#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

class ChassisToggleDriveCommand : public TapComprisedCommand {
public:
    ChassisToggleDriveCommand(
        src::Drivers*,
        ChassisSubsystem*,
        Gimbal::GimbalSubsystem*,
        const SnapSymmetryConfig& snapSymmetryConfig = SnapSymmetryConfig(),
        const TokyoConfig& tokyoConfig = TokyoConfig(),
        bool randomizeSpinRate = false,
        const SpinRandomizerConfig& randomizerConfig = SpinRandomizerConfig());

    void initialize() override;
    void execute() override;

    void end(bool interupted) override;
    bool isReady() override;
    bool isFinished() const override;

    char const* getName() const override { return "Chassis Toggle Drive Command"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    ChassisFollowGimbalCommand followGimbalCommand;
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