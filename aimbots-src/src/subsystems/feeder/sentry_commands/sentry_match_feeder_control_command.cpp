#include "sentry_match_feeder_control_command.hpp"

namespace src::Feeder {

static constexpr int BASE_BURST_LENGTH = 3;
static constexpr int ANNOYED_BURST_LENGTH = 10;

SentryMatchFeederControlCommand::SentryMatchFeederControlCommand(src::Drivers* drivers,
                                                                 FeederSubsystem* feeder,
                                                                 src::Chassis::ChassisMatchStates& chassisState)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      feeder(feeder),
      chassisState(chassisState),
      stopCommand(drivers, feeder),
      burstFireCommand(drivers, feeder, BASE_BURST_LENGTH),
      fullAutoFireCommand(drivers, feeder),
      genericFireCommand(&burstFireCommand)  //
{
    this->comprisedCommandScheduler.registerSubsystem(feeder);
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void SentryMatchFeederControlCommand::initialize() {
    burstFireCommand.setBurstLength(DEFAULT_BURST_LENGTH);
    genericFireCommand = &burstFireCommand;
}

void SentryMatchFeederControlCommand::execute() {
    descheduleIfScheduled(this->comprisedCommandScheduler, &fullAutoFireCommand, true);

    // if (1) {
    if (chassisState != src::Chassis::ChassisMatchStates::EVADE /* && drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::vision::CVState::FIRE*/) {
        auto botData = drivers->refSerial.getRobotData();
        float healthPercentage = static_cast<float>(botData.currentHp) / static_cast<float>(botData.maxHp);

        if (healthPercentage >= 0.50f) {
            burstFireCommand.setBurstLength(BASE_BURST_LENGTH);
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &burstFireCommand);
        } else if (healthPercentage >= 0.33f) {
            burstFireCommand.setBurstLength(ANNOYED_BURST_LENGTH);
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &burstFireCommand);
        } else {
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &fullAutoFireCommand);
        }
    } else {
        descheduleIfScheduled(this->comprisedCommandScheduler, &burstFireCommand, true);
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &stopCommand);
    }

    this->comprisedCommandScheduler.run();
}

void SentryMatchFeederControlCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &burstFireCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &fullAutoFireCommand, interrupted);
}

bool SentryMatchFeederControlCommand::isReady() { return true; }

bool SentryMatchFeederControlCommand::isFinished() const { return false; }

}  // namespace src::Feeder