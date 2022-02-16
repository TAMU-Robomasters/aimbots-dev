#pragma once

#include "drivers.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Shooter {

class ShooterCommand : public TapCommand
{
    public:
    ShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "shooter subsystem"; }

    private:
    src::Drivers* drivers;
    ShooterSubsystem* shooter;
};

}//namespace src::Shooter