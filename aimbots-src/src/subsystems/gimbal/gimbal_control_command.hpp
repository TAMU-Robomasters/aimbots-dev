#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_controller_interface.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <tap/control/command.hpp>

namespace src::Gimbal {

enum gimbalControlMode {
    PATROL,
    CHASE,
    MANUAL
};

class GimbalControlCommand : public tap::control::Command {
   public:
    GimbalControlCommand(src::Drivers*,
                         GimbalSubsystem*,
                         GimbalControllerInterface*,
                         float inputYawSensitivity,
                         float inputPitchSensitivity);

    char const* getName() const override { return "Gimbal Control Command"; }

    void initialize() override;
    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

   private:
    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalControllerInterface* controller;

    float userInputYawSensitivityFactor;
    float userInputPitchSensitivityFactor;

    gimbalControlMode currMode;
};

}  // namespace src::Gimbal