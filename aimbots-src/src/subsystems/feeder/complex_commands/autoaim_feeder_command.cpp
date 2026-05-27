#include "autoaim_feeder_command.hpp"
#include "tap/control/subsystem.hpp"
#include "communicators/jetson/jetson_protocol.hpp"
#include "subsystems/feeder/basic_commands/dual_barrel_feeder_command.hpp"
#include "subsystems/feeder/complex_commands/feeder_limit_command.hpp"
#include "subsystems/feeder/complex_commands/feeder_shot_timing_command.hpp"
#include "subsystems/feeder/control/feeder.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/tools/common_types.hpp"
#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

AutoAimFeederCommand::AutoAimFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    std::array<BarrelID, 2> BARREL_IDS,
    uint8_t projectileBuffer,
    int UNJAM_TIMER_MS)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      dualBarrelFeederCommand(drivers, feeder, refHelper, BARREL_IDS, projectileBuffer, UNJAM_TIMER_MS),
      feederShotTimingCommand(drivers, feeder, refHelper, UNJAM_TIMER_MS)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    comprisedCommandScheduler.registerSubsystem(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void AutoAimFeederCommand::initialize() {
    descheduleIfScheduled(this->comprisedCommandScheduler, &dualBarrelFeederCommand, true);
    descheduleIfScheduled(this->comprisedCommandScheduler, &feederShotTimingCommand, true);
    currentState = FeederState::IDLE;
    noTargetTimer.stop();
}

void AutoAimFeederCommand::execute() {
    bool jetsonOnline = drivers->cvCommunicator.isJetsonOnline();
    Informants::Vision::CVState autoAimState = drivers->cvCommunicator.getLastValidAimMessage().cvState;

    if (!jetsonOnline) {
        descheduleIfScheduled(this->comprisedCommandScheduler, &dualBarrelFeederCommand, true);
        descheduleIfScheduled(this->comprisedCommandScheduler, &feederShotTimingCommand, true);
        currentState = FeederState::IDLE;
        noTargetTimer.stop();
        return;
    }

    switch (currentState) {
        case FeederState::IDLE:
            if (autoAimState == Informants::Vision::CONTINUOUS_FIRE) {
                scheduleIfNotScheduled(this->comprisedCommandScheduler, &dualBarrelFeederCommand);
                currentState = FeederState::CONTINUOUS_FIRE;
            } else if (autoAimState == Informants::Vision::SHOT_TIMING) {
                scheduleIfNotScheduled(this->comprisedCommandScheduler, &feederShotTimingCommand);
                currentState = FeederState::SHOT_TIMING;
            }
            break;

        case FeederState::CONTINUOUS_FIRE:
            if (autoAimState == Informants::Vision::SHOT_TIMING) {
                noTargetTimer.stop();
                descheduleIfScheduled(this->comprisedCommandScheduler, &dualBarrelFeederCommand, true);
                scheduleIfNotScheduled(this->comprisedCommandScheduler, &feederShotTimingCommand);
                currentState = FeederState::SHOT_TIMING;
            } else if (autoAimState == Informants::Vision::NO_TARGET) {
                if (noTargetTimer.isStopped()) noTargetTimer.restart(NO_TARGET_IDLE_TIMEOUT_MS);
                if (noTargetTimer.isExpired()) {
                    descheduleIfScheduled(this->comprisedCommandScheduler, &dualBarrelFeederCommand, true);
                    currentState = FeederState::IDLE;
                    noTargetTimer.stop();
                }
            } else {
                noTargetTimer.stop();
            }
            break;

        case FeederState::SHOT_TIMING:
            if (autoAimState == Informants::Vision::CONTINUOUS_FIRE) {
                noTargetTimer.stop();
                descheduleIfScheduled(this->comprisedCommandScheduler, &feederShotTimingCommand, true);
                scheduleIfNotScheduled(this->comprisedCommandScheduler, &dualBarrelFeederCommand);
                currentState = FeederState::CONTINUOUS_FIRE;
            } else if (autoAimState == Informants::Vision::NO_TARGET) {
                if (noTargetTimer.isStopped()) noTargetTimer.restart(NO_TARGET_IDLE_TIMEOUT_MS);
                if (noTargetTimer.isExpired()) {
                    descheduleIfScheduled(this->comprisedCommandScheduler, &feederShotTimingCommand, true);
                    currentState = FeederState::IDLE;
                    noTargetTimer.stop();
                }
            } else {
                noTargetTimer.stop();
            }
            break;
    }
}

void AutoAimFeederCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &dualBarrelFeederCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &feederShotTimingCommand, interrupted);
}

bool AutoAimFeederCommand::isReady() { return true; }

bool AutoAimFeederCommand::isFinished() const { return false; }

} // namespace src::Feeder

#endif // #ifdef FEEDER_COMPATIBLE


