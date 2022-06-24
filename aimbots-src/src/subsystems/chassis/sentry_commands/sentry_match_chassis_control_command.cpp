#include "sentry_match_chassis_control_command.hpp"

namespace src::Chassis {

static constexpr float MAX_SAFE_DPS = 50.0f;
static constexpr uint32_t EVADE_DURATION_MS = 3000;

SentryMatchChassisControlCommand::SentryMatchChassisControlCommand(src::Drivers* drivers, ChassisSubsystem* chassis, ChassisMatchStates& chassisState)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      chassis(chassis),
      chassisState(chassisState),
      patrolCommand(drivers, chassis),
      evadeCommand(drivers, chassis),
      evadeTimeout(EVADE_DURATION_MS) {
    addSubsystemRequirement(chassis);
    this->comprisedCommandScheduler.registerSubsystem(chassis);
}

void SentryMatchChassisControlCommand::initialize() {
    // chassis->setDefaultCommand(dynamic_cast<TapCommand*>(&patrolCommand));
    if (!comprisedCommandScheduler.isCommandScheduled(&evadeCommand)) {
        chassisState = ChassisMatchStates::PATROL;
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &patrolCommand);
    }
}

float dpsDisplay = 0.0f;

void SentryMatchChassisControlCommand::execute() {
    auto botData = drivers->refSerial.getRobotData();
    float recievedDPS = botData.receivedDps;

    dpsDisplay = recievedDPS;

    if (recievedDPS >= MAX_SAFE_DPS) {
        if (!comprisedCommandScheduler.isCommandScheduled(&evadeCommand)) {
            chassisState = ChassisMatchStates::EVADE;
            this->comprisedCommandScheduler.addCommand(&evadeCommand);
            evadeTimeout.restart(EVADE_DURATION_MS);
        }
    }

    if (evadeTimeout.isExpired()) {
        if (comprisedCommandScheduler.isCommandScheduled(&evadeCommand)) {
            comprisedCommandScheduler.removeCommand(&evadeCommand, true);
        }
    }

    if (!comprisedCommandScheduler.isCommandScheduled(&evadeCommand)) {
        chassisState = ChassisMatchStates::PATROL;
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &patrolCommand);
    }

    this->comprisedCommandScheduler.run();
}

void SentryMatchChassisControlCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &patrolCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &evadeCommand, interrupted);
    chassisState = NONE;
}

bool SentryMatchChassisControlCommand::isReady() { return true; }

bool SentryMatchChassisControlCommand::isFinished() const { return false; }

}  // namespace src::Chassis