#include "sentry_match_feeder_control_command.hpp"

namespace src::Feeder {

static constexpr int STARTING_BURST_LENGTH = 3;
static constexpr int DESPERATE_BURST_LENGTH = 5;

SentryMatchFeederControlCommand::SentryMatchFeederControlCommand(src::Drivers* drivers,
                                                                 FeederSubsystem* feeder)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      feeder(feeder),
      stopCommand(drivers, feeder),
      burstFireCommand(drivers, feeder, STARTING_BURST_LENGTH),
      fullAutoFireCommand(drivers, feeder)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void SentryMatchFeederControlCommand::initialize() {
    feeder->setDefaultCommand(dynamic_cast<TapCommand*>(&stopCommand));
}

void SentryMatchFeederControlCommand::execute() {
    if (!isChassisAccelerating() && isBurstCommandFinished()) {
        auto botData = drivers->refSerial.getRobotData();
        float health_pct = botData.currentHp / (float)botData.maxHp;

        if (health_pct >= 0.50f) {
            descheduleFullAutoIfNeccecary();

            burstFireCommand.setBurstLength(DEFAULT_BURST_LENGTH);
            comprisedCommandScheduler.addCommand(dynamic_cast<TapCommand*>(&burstFireCommand));
        } else if (health_pct >= 0.33f) {
            descheduleFullAutoIfNeccecary();

            burstFireCommand.setBurstLength(DESPERATE_BURST_LENGTH);
            comprisedCommandScheduler.addCommand(dynamic_cast<TapCommand*>(&burstFireCommand));
        } else {
            if (!comprisedCommandScheduler.isCommandScheduled(dynamic_cast<TapCommand*>(&fullAutoFireCommand)))
                comprisedCommandScheduler.addCommand(dynamic_cast<TapCommand*>(&fullAutoFireCommand));
        }
    } else {
        // We don't want to run full auto while we're driving.
        descheduleFullAutoIfNeccecary();
    }
}

void SentryMatchFeederControlCommand::end(bool interrupted) { UNUSED(interrupted); }

bool SentryMatchFeederControlCommand::isReady() { return true; }

bool SentryMatchFeederControlCommand::isFinished() const { return false; }

}  // namespace src::Feeder