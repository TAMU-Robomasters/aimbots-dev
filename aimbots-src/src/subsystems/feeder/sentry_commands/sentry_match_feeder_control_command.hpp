#pragma once

#include "drivers.hpp"
#include "subsystems/chassis/sentry_commands/sentry_match_chassis_control_command.hpp"
#include "subsystems/feeder/burst_feeder_command.hpp"
#include "subsystems/feeder/feeder.hpp"
#include "subsystems/feeder/full_auto_feeder_command.hpp"
#include "subsystems/feeder/stop_feeder_command.hpp"
#include "utils/common_types.hpp"

namespace src::Feeder {

enum FeederMatchStates {
    ANNOYED = 0,
    AGGRESSIVE = 1,
    FURIOUS = 2,
};

static constexpr float SENTRY_SHOOTER_MAX_ACCEL = 2.5f;

class SentryMatchFeederControlCommand : public TapComprisedCommand {
   public:
    SentryMatchFeederControlCommand(src::Drivers*, FeederSubsystem*, src::Chassis::ChassisMatchStates& chassisState);

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Feeder Control Command"; }

   private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;

    src::Chassis::ChassisMatchStates& chassisState;

    StopFeederCommand stopCommand;
    BurstFeederCommand burstFireCommand;
    FullAutoFeederCommand fullAutoFireCommand;

    TapCommand* genericFireCommand;
};

}  // namespace src::Feeder