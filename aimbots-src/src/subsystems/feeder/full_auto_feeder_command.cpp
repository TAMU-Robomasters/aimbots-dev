#include "full_auto_feeder_command.hpp"

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    float speed,
    float unjamSpeed,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      speed(speed),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      unjamSpeed(-unjamSpeed)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

bool isCommandRunningDisplay = false;

void FullAutoFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

uint16_t lastHeatDisplay = 0;
uint16_t heatLimitDisplay = 0;

void FullAutoFeederCommand::execute() {
    isCommandRunningDisplay = true;
    if (fabs(feeder->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
        feeder->setTargetRPM(unjamSpeed);
        unjamTimer.restart(UNJAM_TIMER_MS);
    }

    if (unjamTimer.execute()) {
        feeder->setTargetRPM(speed);
        startupThreshold.restart(500);
    }

    lastHeatDisplay = refHelper->getCurrBarrelHeat();
    heatLimitDisplay = refHelper->getCurrBarrelLimit();
}

void FullAutoFeederCommand::end(bool) { feeder->setTargetRPM(0.0f); isCommandRunningDisplay = false; }

bool FullAutoFeederCommand::isReady() { return refHelper->canCurrBarrelShootSafely(); }

bool FullAutoFeederCommand::isFinished() const { return !refHelper->canCurrBarrelShootSafely(); }

}  // namespace src::Feeder