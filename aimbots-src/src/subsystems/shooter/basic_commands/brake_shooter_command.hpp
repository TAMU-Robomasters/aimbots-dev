#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/shooter/control/shooter.hpp"
#include "utils/tools/common_types.hpp"


#include "drivers.hpp"

#ifdef SHOOTER_COMPATIBLE

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

#endif //#ifdef SHOOTER_COMPATIBLE