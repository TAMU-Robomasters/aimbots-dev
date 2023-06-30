#include "burst_feeder_command.hpp"

namespace src::Feeder {
BurstFeederCommand::BurstFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    float speed,
    float acceptableHeatThreshold,
    int burstLength)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      startingTotalBallCount(0),
      burstLength(burstLength) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void BurstFeederCommand::initialize() { startingTotalBallCount = feeder->getTotalLimitCount(); }

void BurstFeederCommand::execute() { feeder->setTargetRPM(speed); }

void BurstFeederCommand::end(bool) { feeder->setTargetRPM(0); }

bool BurstFeederCommand::isReady() { return refHelper->isCurrBarrelHeatUnderLimit(acceptableHeatThreshold); }

bool BurstFeederCommand::isFinished() const {
    int elapsedTotal = feeder->getTotalLimitCount() - startingTotalBallCount;
    return (elapsedTotal >= burstLength) || !refHelper->isCurrBarrelHeatUnderLimit(acceptableHeatThreshold);
}

}  // namespace src::Feeder