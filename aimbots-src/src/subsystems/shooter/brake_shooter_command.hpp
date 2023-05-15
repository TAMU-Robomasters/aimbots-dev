#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/shooter/shooter.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Shooter {

class BrakeShooterCommand : public TapCommand {
public:
    BrakeShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter, float brakePower);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "shooter brake command"; }

private:
    src::Drivers* drivers;
    ShooterSubsystem* shooter;
    StockPID brakePID;
};

}  // namespace src::Shooter