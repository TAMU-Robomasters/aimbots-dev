#pragma once

#include <tap/control/command.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <drivers.hpp>

namespace src::Gimbal {

class GimbalControlCommand : public tap::control::Command {
public:
    GimbalControlCommand(src::Drivers*, GimbalSubsystem*);

    char const* getName() const override { return "Gimbal Control Command"; }
    bool isReady() override { return true; }

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isFinished() const override;

private:
    src::Drivers*    drivers;
    GimbalSubsystem* gimbal;
};

}  // namespace src::Gimbal