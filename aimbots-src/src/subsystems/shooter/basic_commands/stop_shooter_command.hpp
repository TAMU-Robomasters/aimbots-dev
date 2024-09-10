#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/shooter/control/shooter.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {

class StopShooterCommand : public TapCommand {
public:
    StopShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "shooter stop command"; }

private:
    src::Drivers* drivers;
    ShooterSubsystem* shooter;
};

}  // namespace src::Shooter

#endif //#ifdef SHOOTER_COMPATIBLE