#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/shooter/brake_shooter_command.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/stop_shooter_command.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {

class StopShooterComprisedCommand : public TapComprisedCommand {
public:
    StopShooterComprisedCommand(src::Drivers* drivers, ShooterSubsystem* shooter);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Slow To Stop Command"; }

private:
    src::Drivers* drivers;
    ShooterSubsystem* shooter;

    BrakeShooterCommand brake_command;
    StopShooterCommand stop_command;
    bool brakeFinished;
};

}  // namespace src::Shooter

#endif //#ifdef SHOOTER_COMPATIBLE
