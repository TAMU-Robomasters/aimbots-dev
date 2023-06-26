#include "burst_indexer_command.hpp"

namespace src::Indexer {
BurstIndexerCommand::BurstIndexerCommand(
    src::Drivers* drivers,
    IndexerSubsystem* indexer,
    float speed,
    float acceptableHeatThreshold,
    int burstLength)
    : drivers(drivers),
      indexer(indexer),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      startingTotalBallCount(0),
      burstLength(burstLength) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(indexer));
}

void BurstIndexerCommand::initialize() { startingTotalBallCount = indexer->getTotalLimitCount(); }

void BurstIndexerCommand::execute() { indexer->setTargetRPM(speed); }

void BurstIndexerCommand::end(bool) { indexer->setTargetRPM(0); }

bool BurstIndexerCommand::isReady() { return indexer->isBarrelHeatAcceptable(acceptableHeatThreshold); }

bool BurstIndexerCommand::isFinished() const {
    int elapsedTotal = indexer->getTotalLimitCount() - startingTotalBallCount;
    return (elapsedTotal >= burstLength) || !indexer->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

}  // namespace src::Indexer