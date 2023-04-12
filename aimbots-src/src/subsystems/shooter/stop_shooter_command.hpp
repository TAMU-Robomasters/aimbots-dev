#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/shooter/shooter.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

#include "drivers.hpp"

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