#pragma once

#include <tap/control/command.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp>
#include <drivers.hpp>

namespace src::Gimbal {

class GimbalControlCommand : public tap::control::Command {
public:
    GimbalControlCommand(src::Drivers*, GimbalSubsystem*, GimbalChassisRelativeController*);

    char const* getName() const override { return "Gimbal Control Command"; }

    void initialize() override;
    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

private:
    src::Drivers*    drivers;
    GimbalSubsystem* gimbal;

    GimbalChassisRelativeController* controller;

    uint32_t previousTime;
};

}  // namespace src::Gimbal