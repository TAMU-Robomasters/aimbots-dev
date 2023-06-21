#pragma once

#include "tap/algorithms/ballistics.hpp"
#include "tap/control/command.hpp"

#include "informants/enemy_data_conversion.hpp"
#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/controllers/gimbal_field_relative_controller.hpp"
#include "subsystems/gimbal/gimbal.hpp"

#include "drivers.hpp"

namespace src::Utils::Ballistics {
class BallisticsSolver;

const modm::Pair<float, float> TARGET_DISTANCE_TO_YAW_VELOCITY_LIMITS[2] = {{0.5f, 3.0f}, {5.0f, 10.0f}};  // m, rad/s

const modm::interpolation::Linear<modm::Pair<float, float>> YAW_VELOCITY_LIMITER(TARGET_DISTANCE_TO_YAW_VELOCITY_LIMITS, 2);
}  // namespace src::Utils::Ballistics
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

    void setIgnoreQuickTurn(bool ignore) {ignoreQuickTurns = ignore;}

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

private:
    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalControllerInterface* controller;

    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver;

    src::Informants::Vision::PlateKinematicState data;

    bool wasQPressed = false;
    bool wasEPressed = false;

    bool ignoreQuickTurns = false;
};

}  // namespace src::Gimbal