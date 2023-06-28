#pragma once

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_follow_gimbal_command.hpp"
#include "subsystems/chassis/chassis_tokyo_command.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "utils/common_types.hpp"

#include "drivers.hpp"

namespace src::Chassis {

class ChassisToggleDriveCommand : public TapComprisedCommand {
public:
    ChassisToggleDriveCommand(
        src::Drivers*,
        ChassisSubsystem*,
        Gimbal::GimbalSubsystem*,
        uint8_t numSnapPositions = 1,
        float starterAngle = 0.0f,
        bool randomizeSpinRate = false,
        const ToykoRandomizerConfig& randomizerConfig = ToykoRandomizerConfig());

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