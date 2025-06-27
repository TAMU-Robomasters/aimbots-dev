#pragma once

#include "subsystems/gimbal/basic_commands/gimbal_chase_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_field_relative_control_command.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "subsystems/gimbal/control/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

namespace src::Gimbal {

class GimbalToggleAimCommand : public TapComprisedCommand {
public:
    GimbalToggleAimCommand(
        src::Drivers*,
        GimbalSubsystem*,
        GimbalFieldRelativeController*,
        GimbalFieldRelativeController*,
        src::Utils::RefereeHelperTurreted*,
        src::Utils::Ballistics::BallisticsSolver*,
        float defaultLaunchSpeed,
        std::optional<float> quickTurnOffset = std::nullopt);

    void initialize() override;
    void execute() override;

    void end(bool interupted) override;
    bool isReady() override;
    bool isFinished() const override;

    char const* getName() const override { return "Gimbal Toggle Aiming Command"; }

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;

    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver;

    GimbalChaseCommand gimbalCVCommand;
    GimbalFieldRelativeControlCommand gimbalFreeAimCommand;

    MilliTimeout ignoreQuickTurn;
};

}  // namespace src::Gimbal