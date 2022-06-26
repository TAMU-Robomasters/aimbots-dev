#include "utils/robot_specific_inc.hpp"
#ifdef TARGET_SENTRY

#include "sentry_match_firing_control_command.hpp"

namespace src::Control {

static constexpr int BASE_BURST_LENGTH = 3;
static constexpr int ANNOYED_BURST_LENGTH = 10;

SentryMatchFiringControlCommand::SentryMatchFiringControlCommand(src::Drivers* drivers,
                                                                 FeederSubsystem* feeder,
                                                                 ShooterSubsystem* shooter,
                                                                 src::Chassis::ChassisMatchStates& chassisState)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      feeder(feeder),
      shooter(shooter),
      chassisState(chassisState),
      stopFeederCommand(drivers, feeder),
      burstFeederCommand(drivers, feeder, BASE_BURST_LENGTH),
      fullAutoFeederCommand(drivers, feeder),
      stopShooterCommand(drivers, shooter),
      runShooterCommand(drivers, shooter)  //
{
    this->comprisedCommandScheduler.registerSubsystem(feeder);
    this->comprisedCommandScheduler.registerSubsystem(shooter);
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void SentryMatchFiringControlCommand::initialize() {
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &stopFeederCommand);
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &runShooterCommand);
}

void SentryMatchFiringControlCommand::execute() {
    // descheduleIfScheduled(this->comprisedCommandScheduler, &fullAutoFeederCommand, true);

    // if (1) {
    if (drivers->cvCommunicator.isJetsonOnline()) {
        if (chassisState != src::Chassis::ChassisMatchStates::EVADE && drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::vision::CVState::FIRE) {
            // auto botData = drivers->refSerial.getRobotData();
            // float healthPercentage = static_cast<float>(botData.currentHp) / static_cast<float>(botData.maxHp);

            // if (healthPercentage >= 0.50f) {
            //     burstFeederCommand.setBurstLength(BASE_BURST_LENGTH);
            //     scheduleIfNotScheduled(this->comprisedCommandScheduler, &burstFeederCommand);
            // } else if (healthPercentage >= 0.33f) {
            //     burstFeederCommand.setBurstLength(ANNOYED_BURST_LENGTH);
            //     scheduleIfNotScheduled(this->comprisedCommandScheduler, &burstFeederCommand);
            // } else {
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &fullAutoFeederCommand);
            // }
        } else {
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &stopFeederCommand);
        }
    } else {
        // descheduleIfScheduled(this->comprisedCommandScheduler, &burstFeederCommand, true);
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &stopFeederCommand);
    }

    this->comprisedCommandScheduler.run();
}

void SentryMatchFiringControlCommand::end(bool interrupted) {
    // descheduleIfScheduled(this->comprisedCommandScheduler, &burstFeederCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &fullAutoFeederCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &stopFeederCommand, interrupted);
}

bool SentryMatchFiringControlCommand::isReady() { return true; }

bool SentryMatchFiringControlCommand::isFinished() const { return false; }

}  // namespace src::Control

#endif