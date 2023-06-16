#include "auto_agitator_indexer_command.hpp"

namespace src::Feeder {

AutoAgitatorIndexerCommand::AutoAgitatorIndexerCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelper* refHelper,
    float speed,
    float acceptableHeatThreshold)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      unjamSpeed(-3000.0f)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void AutoAgitatorIndexerCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

void AutoAgitatorIndexerCommand::execute() {
    if (fabs(feeder->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
        feeder->setTargetRPM(unjamSpeed);
        unjamTimer.restart(UNJAM_TIMER_MS);
    }

    if (unjamTimer.execute()) {
        feeder->setTargetRPM(speed);
        startupThreshold.restart(500);
    }
}

void AutoAgitatorIndexerCommand::end(bool) { feeder->setTargetRPM(0.0f); }

bool AutoAgitatorIndexerCommand::isReady() {
    return (refHelper->isBarrelHeatUnderLimit(acceptableHeatThreshold));
}

bool AutoAgitatorIndexerCommand::isFinished() const {
    return (!refHelper->isBarrelHeatUnderLimit(acceptableHeatThreshold));
}

}  // namespace src::Feeder