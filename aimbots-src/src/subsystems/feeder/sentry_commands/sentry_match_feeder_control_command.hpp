#pragma once

#include "drivers.hpp"
#include "utils/common_types.hpp"
#include "subsystems/feeder/feeder.hpp"
#include "subsystems/feeder/stop_feeder_command.hpp"
#include "subsystems/feeder/burst_feeder_command.hpp"
#include "subsystems/feeder/full_auto_feeder_command.hpp"

namespace src::Feeder {

static constexpr float SENTRY_SHOOTER_MAX_ACCEL = 2.5f;

class SentryMatchFeederControlCommand : public TapComprisedCommand {
   public:
    SentryMatchFeederControlCommand(src::Drivers*, FeederSubsystem*);

    void initialize() override;
    void execute() override;
    
    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Feeder Control Command"; }

    inline bool isChassisAccelerating() const {
        return (abs(drivers->bmi088.getAx()) >= SENTRY_SHOOTER_MAX_ACCEL)
            || (abs(drivers->bmi088.getAy()) >= SENTRY_SHOOTER_MAX_ACCEL);
    }

    inline bool isBurstCommandFinished() const {
        return burstFireCommand.isFinished();
    }

    inline void descheduleFullAutoIfNeccecary() {
        if (comprisedCommandScheduler.isCommandScheduled(dynamic_cast<TapCommand*>(&fullAutoFireCommand))) {
            comprisedCommandScheduler.removeCommand(dynamic_cast<TapCommand*>(&fullAutoFireCommand), true);
        }
    }

   private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;

    StopFeederCommand stopCommand;
    BurstFeederCommand burstFireCommand;
    FullAutoFeederCommand fullAutoFireCommand;
};

}