#include "sentry_match_gimbal_control_command.hpp"

#ifdef ALL_SENTRIES

namespace src::Gimbal {

SentryMatchGimbalControlCommand::SentryMatchGimbalControlCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbal,
    GimbalFieldRelativeController* gimbalController,
    src::Utils::RefereeHelperTurreted* refHelper,
    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver,
    GimbalPatrolConfig* patrolConfig,
    int chaseTimeoutMillis)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      gimbal(gimbal),
      controller(gimbalController),
      ballisticsSolver(ballisticsSolver),
      patrolCommand(drivers, gimbal, controller, patrolConfig, chassisState),
      chaseCommand(drivers, gimbal, controller, refHelper, ballisticsSolver, 30.0f),
      chassisState(chassisState), 
      chaseTimeout(0),
      chaseTimeoutMillis(chaseTimeoutMillis)  
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

    if (drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::Vision::FOUND ||
        drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::Vision::FIRE) {
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

#endif