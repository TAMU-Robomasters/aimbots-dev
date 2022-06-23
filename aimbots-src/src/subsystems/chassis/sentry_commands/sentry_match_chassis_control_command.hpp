#pragma once

#include "drivers.hpp"
#include "utils/common_types.hpp"
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/sentry_commands/chassis_rail_bounce_command.hpp"
#include "subsystems/chassis/sentry_commands/chassis_rail_evade_command.hpp"

namespace src::Chassis {

class SentryMatchChassisControlCommand : public TapComprisedCommand {
   public:
    SentryMatchChassisControlCommand(src::Drivers*, ChassisSubsystem*);

    void initialize() override;
    void execute() override;
    
    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Feeder Control Command"; }

   private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    ChassisRailBounceCommand patrolCommand;
    ChassisRailEvadeCommand evadeCommand;

    MilliTimeout evadeTimeout;
};

}