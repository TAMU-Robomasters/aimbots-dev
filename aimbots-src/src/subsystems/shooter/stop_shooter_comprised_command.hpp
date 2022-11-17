#pragma once
#ifndef ENGINEER

#include "tap/control/subsystem.hpp"

#include "subsystems/shooter/brake_shooter_command.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/stop_shooter_command.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

#include "drivers.hpp"

//#ifndef TARGET_ENGINEER

namespace src::Shooter {

class StopShooterComprisedCommand : public TapComprisedCommand {
public:
#ifndef ENGINEER

    StopShooterComprisedCommand(src::Drivers* drivers, ShooterSubsystem* shooter);
#endif
    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Slow To Stop Command"; }

private:
#ifndef ENGINEER

    src::Drivers* drivers;
    ShooterSubsystem* shooter;

    BrakeShooterCommand brake_command;
    StopShooterCommand stop_command;
    bool brakeFinished;
#endif
};

}  // namespace src::Shooter

#endif