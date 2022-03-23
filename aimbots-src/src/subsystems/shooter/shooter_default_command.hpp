#pragma once

#include "drivers.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

//#ifndef TARGET_ENGINEER

namespace src::Shooter {


class ShooterDefaultCommand : public TapCommand
{
    public:
    ShooterDefaultCommand(src::Drivers* drivers, ShooterSubsystem* shooter);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "shooter default command"; }

    private:
    src::Drivers* drivers;
    ShooterSubsystem* shooter;
};

}//namespace src::Shooter

//#endif