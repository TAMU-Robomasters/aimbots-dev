#include "sentry_match_gimbal_control_command.hpp"

namespace src::Gimbal {

SentryMatchGimbalControlCommand::SentryMatchGimbalControlCommand(src::Drivers* drivers, GimbalSubsystem* gimbal, GimbalChassisRelativeController* gimbalController, int chaseTimeoutMillis)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      gimbal(gimbal),
      controller(gimbalController),
      patrolCommand(drivers, gimbal, controller),
      chaseCommand(drivers, gimbal, controller),
      chaseTimeout(0),
      chaseTimeoutMillis(chaseTimeoutMillis)  //
{
    addSubsystemRequirement(gimbal);
    this->comprisedCommandScheduler.registerSubsystem(gimbal);
}

void SentryMatchGimbalControlCommand::initialize() {
    if (!comprisedCommandScheduler.isCommandScheduled(&patrolCommand)) {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &patrolCommand);
    }
}

void SentryMatchGimbalControlCommand::execute() {
    if (!drivers->cvCommunicator.isJetsonOnline()) {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &patrolCommand);
        this->comprisedCommandScheduler.run();
        return;
    }

    if (drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::vision::FOUND ||
        drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::vision::FIRE) {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &chaseCommand);
        chaseTimeout.restart(chaseTimeoutMillis);
    }
    if (chaseTimeout.isExpired()) {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &patrolCommand);
    }

    this->comprisedCommandScheduler.run();
}

void SentryMatchGimbalControlCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &patrolCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &chaseCommand, interrupted);
}

bool SentryMatchGimbalControlCommand::isReady() { return true; }

bool SentryMatchGimbalControlCommand::isFinished() const { return false; }

}  // namespace src::Gimbal