#pragma once

#include "tap/algorithms/ballistics.hpp"
#include "tap/control/command.hpp"

#include "informants/enemy_data_conversion.hpp"
#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/gimbal.hpp"

#include "drivers.hpp"

namespace src::Utils::Ballistics {
class BallisticsSolver;
}
namespace src::Gimbal {

class GimbalChaseCommand : public tap::control::Command {
public:
    GimbalChaseCommand(
        src::Drivers*,
        GimbalSubsystem*,
        GimbalControllerInterface*,
        src::Utils::Ballistics::BallisticsSolver*);

    char const* getName() const override { return "Gimbal Chase Command"; }

    void initialize() override;
    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

private:
    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalControllerInterface* controller;

    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver;

    src::Informants::vision::plateKinematicState data;
    GimbalSubsystem::aimAngles aimAtAngles;
};

}  // namespace src::Gimbal