#include "full_auto_feeder_command.hpp"

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelper* refHelper,
    float speed,
    float acceptableHeatThreshold,
    int UNJAM_TIMER_MS)
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

void FullAutoFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

void FullAutoFeederCommand::execute() {
    if (fabs(feeder->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
        feeder->setTargetRPM(unjamSpeed);
        unjamTimer.restart(UNJAM_TIMER_MS);
    }

    if (unjamTimer.execute()) {
        feeder->setTargetRPM(speed);
        startupThreshold.restart(500);
    }
}

void FullAutoFeederCommand::end(bool) { feeder->setTargetRPM(0.0f); }

bool FullAutoFeederCommand::isReady() {
    return (refHelper->isBarrelHeatUnderLimit(acceptableHeatThreshold));
}

bool FullAutoFeederCommand::isFinished() const {
    return (!refHelper->isBarrelHeatUnderLimit(acceptableHeatThreshold));
}

}  // namespace src::Feeder