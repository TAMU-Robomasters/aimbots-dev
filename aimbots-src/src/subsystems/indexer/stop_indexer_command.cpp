#include "stop_indexer_command.hpp"

#ifndef ENGINEER
namespace src::Indexer {
StopIndexerCommand::StopIndexerCommand(src::Drivers* drivers, IndexerSubsystem* indexer) : drivers(drivers), indexer(indexer) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(indexer));
}

void StopIndexerCommand::initialize() { indexer->setTargetRPM(0); }

void StopIndexerCommand::execute() { indexer->setTargetRPM(0); }

void StopIndexerCommand::end(bool interrupted) { UNUSED(interrupted); }

bool StopIndexerCommand::isReady() { return true; }

bool StopIndexerCommand::isFinished() const { return false; }

}  // namespace src::Indexer
#endif