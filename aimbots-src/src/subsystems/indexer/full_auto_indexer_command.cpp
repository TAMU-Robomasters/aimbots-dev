#include "full_auto_indexer_command.hpp"

namespace src::Indexer {
FullAutoIndexerCommand::FullAutoIndexerCommand(src::Drivers* drivers, IndexerSubsystem* indexer, src::Utils::RefereeHelperTurreted* refHelper, float speed, float acceptableHeatThreshold)
    : drivers(drivers),
      indexer(indexer),
      refHelper(refHelper),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      unjamSpeed(-3000.0f)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(indexer));
}

void FullAutoIndexerCommand::initialize() {
    //indexer->setTargetRPM(0.0f);
    //startupThreshold.restart(500);  // delay to wait before attempting unjam
    //unjamTimer.restart(0);
}

void FullAutoIndexerCommand::execute() {

    indexer->setTargetRPM(speed);
    /*if (fabs(indexer->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
        indexer->setTargetRPM(unjamSpeed);
        unjamTimer.restart(175);
    }

    if (unjamTimer.execute()) {
        
        startupThreshold.restart(500);
    }*/
}

void FullAutoIndexerCommand::end(bool) { indexer->setTargetRPM(0.0f); }

bool FullAutoIndexerCommand::isReady() { return refHelper->canCurrBarrelShootSafely(); }

bool FullAutoIndexerCommand::isFinished() const { return !refHelper->canCurrBarrelShootSafely(); }

}  // namespace src::Indexer
