#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <tap/algorithms/ballistics.hpp>
#include <tap/control/command.hpp>

#include "src/informants/enemy_data_conversion.hpp"

namespace src::Utils {
class BallisticsSolver;
}
namespace src::Gimbal {

class GimbalChaseCommand : public tap::control::Command {
public:
    GimbalChaseCommand(src::Drivers*, GimbalSubsystem*, GimbalControllerInterface*, src::Utils::BallisticsSolver*);

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

    src::Utils::BallisticsSolver* ballisticsSolver;

    src::Informants::vision::plateKinematicState data;
    GimbalSubsystem::aimAngles aimAtAngles;
};

}  // namespace src::Gimbal