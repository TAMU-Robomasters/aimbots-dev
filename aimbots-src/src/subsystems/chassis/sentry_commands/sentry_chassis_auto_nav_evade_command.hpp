#pragma once

#include "subsystems/chassis/sentry_commands/sentry_match_chassis_control_command.hpp"

namespace src::Chassis {

class SentryChassisAutoNavEvadeCommand : public TapCommand {
public:
    SentryChassisAutoNavEvadeCommand(src::Drivers* drivers, ChassisSubsystem* chassis);

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Chassis Evade Command"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
}

}  // namespace src::Chassis