#include "sentry_match_chassis_control_command.hpp"

namespace src::Chassis {

static constexpr float MAX_SAFE_DPS = 10.0f;
static constexpr uint32_t EVADE_DURATION_MS = 3000;

SentryMatchChassisControlCommand::SentryMatchChassisControlCommand(src::Drivers* drivers, ChassisSubsystem* chassis)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      chassis(chassis),
      patrolCommand(drivers, chassis),
      evadeCommand(drivers, chassis),
      evadeTimeout(EVADE_DURATION_MS)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void SentryMatchChassisControlCommand::initialize() {
    chassis->setDefaultCommand(dynamic_cast<TapCommand*>(&patrolCommand));
}

void SentryMatchChassisControlCommand::execute() {
    auto botData = drivers->refSerial.getRobotData();
    float recievedDPS = botData.receivedDps;

    if (recievedDPS > MAX_SAFE_DPS) {
        if(!comprisedCommandScheduler.isCommandScheduled(dynamic_cast<TapCommand*>(&evadeCommand))) {
            comprisedCommandScheduler.addCommand(dynamic_cast<TapCommand*>(&evadeCommand));
            evadeTimeout.restart(EVADE_DURATION_MS);
        }
    }

    if (evadeTimeout.isExpired()) {
        if (comprisedCommandScheduler.isCommandScheduled(dynamic_cast<TapCommand*>(&evadeCommand)))
            comprisedCommandScheduler.removeCommand(dynamic_cast<TapCommand*>(&evadeCommand), true);
    }
}

void SentryMatchChassisControlCommand::end(bool interrupted) { UNUSED(interrupted); }

bool SentryMatchChassisControlCommand::isReady() { return true; }

bool SentryMatchChassisControlCommand::isFinished() const { return true; }

}