#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <tap/control/command.hpp>

namespace src::Gimbal {

class GimbalControlCommand : public tap::control::Command {
   public:
    GimbalControlCommand(src::Drivers*,
                         GimbalSubsystem*,
                         GimbalChassisRelativeController*,
                         float inputYawSensitivity,
                         float inputPitchSensitiity);

    char const* getName() const override { return "Gimbal Control Command"; }

    void initialize() override;
    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

   private:
    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalChassisRelativeController* controller;

    float userInputYawSensitivityFactor;
    float userInputPitchSensitivityFactor;
};

}  // namespace src::Gimbal