#pragma once

#include <tap/control/command.hpp>

#include <drivers.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp>

namespace src::Gimbal {

class GimbalCVCommand : public tap::control::Command {
   public:
    GimbalCVCommand(src::Drivers*,
                    GimbalSubsystem*,
                    GimbalChassisRelativeController*);

    char const* getName() const override { return "Gimbal CV Control Command"; }

    void initialize() override;
    void execute() override;

    inline bool isReady() override { return true; }
    inline bool isFinished() const override { return false; }
    inline void end(bool interrupted) override {
        (void)interrupted;
        gimbal->setYawMotorOutput(0);
        gimbal->setPitchMotorOutput(0);
    }

   private:
    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalChassisRelativeController* controller;
};

}