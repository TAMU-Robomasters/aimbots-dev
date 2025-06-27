#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/control/gimbal_controller_interface.hpp>
#include <subsystems/gimbal/control/gimbal.hpp>
#include <tap/control/command.hpp>
#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"

#ifdef GIMBAL_COMPATIBLE

namespace src::Gimbal {

class GimbalFieldRelativeControlCommand : public tap::control::Command {
public:
    GimbalFieldRelativeControlCommand(
        src::Drivers*,
        GimbalSubsystem*,
        GimbalFieldRelativeController*,
        std::optional<float> quickTurnOffset = std::nullopt);

    char const* getName() const override { return "Gimbal Control Command"; }

    void initialize() override;
    void execute() override;

    void setIgnoreQuickTurn(bool ignore) { ignoreQuickTurns = ignore; }

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

private:
    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalFieldRelativeController* controller;

    std::optional<float> quickTurnOffset;

    bool wasQPressed = false;
    bool wasEPressed = false;

    bool ignoreQuickTurns = false;

    bool forceYawOffset = false;
    bool wasVPRessed = false;
};

}  // namespace src::Gimbal

#endif