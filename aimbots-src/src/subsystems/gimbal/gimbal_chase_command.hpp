#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <tap/control/command.hpp>

namespace src::Gimbal {

class GimbalChaseCommand : public tap::control::Command {
   public:
    GimbalChaseCommand(src::Drivers*,
                       GimbalSubsystem*,
                       GimbalControllerInterface*);

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
};

}  // namespace src::Gimbal