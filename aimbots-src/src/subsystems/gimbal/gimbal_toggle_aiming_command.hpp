#pragma once

#include "subsystems/gimbal/gimbal.hpp"
#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/controllers/gimbal_field_relative_controller.hpp"
#include "subsystems/gimbal/gimbal_chase_command.hpp"
#include "subsystems/gimbal/gimbal_field_relative_control_command.hpp"

#include "utils/common_types.hpp"

#include "drivers.hpp"

namespace src::Gimbal {

class GimbalToggleAimCommand : public TapComprisedCommand {
public:
    GimbalToggleAimCommand(src::Drivers*, 
                            GimbalSubsystem*, 
                            GimbalControllerInterface*,
                            src::Utils::Ballistics::BallisticsSolver*);

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