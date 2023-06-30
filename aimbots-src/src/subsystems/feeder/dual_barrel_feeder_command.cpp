#include "dual_barrel_feeder_command.hpp"

namespace src::Feeder {

DualBarrelFeederCommand::DualBarrelFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    bool& barrelMovingFlag,
    float speed,
    float acceptableHeatThreshold,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      barrelMovingFlag(barrelMovingFlag),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      unjamSpeed(-3000.0f)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void DualBarrelFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

void DualBarrelFeederCommand::execute() {
    if (fabs(feeder->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
        feeder->setTargetRPM(unjamSpeed);
        unjamTimer.restart(UNJAM_TIMER_MS);
    }

    if (unjamTimer.execute()) {
        feeder->setTargetRPM(speed);
        startupThreshold.restart(500);
    }
}

void DualBarrelFeederCommand::end(bool) { feeder->setTargetRPM(0.0f); }

bool DualBarrelFeederCommand::isReady() {
    return (refHelper->isCurrBarrelHeatUnderLimit(acceptableHeatThreshold) && !barrelMovingFlag);
}

bool DualBarrelFeederCommand::isFinished() const {
    return (!refHelper->isCurrBarrelHeatUnderLimit(acceptableHeatThreshold) || barrelMovingFlag);
}

}  // namespace src::Feeder