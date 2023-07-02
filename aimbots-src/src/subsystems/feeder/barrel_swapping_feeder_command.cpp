#include "barrel_swapping_feeder_command.hpp"

namespace src::Feeder {

BarrelSwappingFeederCommand::BarrelSwappingFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    bool& barrelMovingFlag,
    float speed,
    float unjamSpeed,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      barrelMovingFlag(barrelMovingFlag),
      speed(speed),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      unjamSpeed(-unjamSpeed)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void BarrelSwappingFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

void BarrelSwappingFeederCommand::execute() {
    if (refHelper->canCurrBarrelShootSafely() && !barrelMovingFlag){
        if (fabs(feeder->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
            feeder->setTargetRPM(unjamSpeed);
            unjamTimer.restart(UNJAM_TIMER_MS);
        }

        if (unjamTimer.execute()) {
            feeder->setTargetRPM(speed);
            startupThreshold.restart(500);
        }
    }
    else {
        feeder->setTargetRPM(0.0f);
        unjamTimer.restart(0);
    }
}

void BarrelSwappingFeederCommand::end(bool) { feeder->setTargetRPM(0.0f); }

bool BarrelSwappingFeederCommand::isReady() {
    return (refHelper->canCurrBarrelShootSafely() && !barrelMovingFlag);
}

bool BarrelSwappingFeederCommand::isFinished() const {
    // return (!refHelper->isCurrBarrelHeatUnderLimit(acceptableHeatThreshold) || barrelMovingFlag);
    return false;
}


}  // namespace src::Feeder