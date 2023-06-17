#include "auto_agitator_indexer_command.hpp"

namespace src::Feeder {

AutoAgitatorIndexerCommand::AutoAgitatorIndexerCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Indexer::IndexerSubsystem* indexer,
    src::Utils::RefereeHelper* refHelper,
    float feederSpeed,
    float indexerSpeed,
    float acceptableHeatThreshold,
    int UNJAM_TIMER_MS,
    int MAX_UNJAM_COUNT)
    : drivers(drivers),
      feeder(feeder),
      indexer(indexer),
      refHelper(refHelper),
      feederSpeed(feederSpeed),
      indexerSpeed(indexerSpeed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      MAX_UNJAM_COUNT(MAX_UNJAM_COUNT),
      unjamSpeed(-3000.0f)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void AutoAgitatorIndexerCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

//The plan
//Run this either as a default command, or paired to a remote switch that should be "always on" (like how shooter flywheels are)
//Read from an additional remote input within this command, as the "shoot" trigger

//While command is running, run agitator at full power
//After reaching an unjam, count it.
//After attempting unjam 3 times, if problem persists, halt (assume fully loaded)
//Read a different remote input to reset the unjam counter (force reload, essentially)
//Once the "shoot" trigger is pressed, start moving the indexer, and reset the unjam counter


void AutoAgitatorIndexerCommand::execute() {
    if (fabs(feeder->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
        feeder->setTargetRPM(unjamSpeed);
        unjamTimer.restart(UNJAM_TIMER_MS);
    }

    if (unjamTimer.execute() && unjamming_count < MAX_UNJAM_COUNT) {
        feeder->setTargetRPM(feederSpeed);
        startupThreshold.restart(500);
        unjamming_count++;
    }

    if (unjamming_count >= MAX_UNJAM_COUNT) {
        feeder->setTargetRPM(0.0f);
    }

    if (drivers->remote.getMouseL() && refHelper->isBarrelHeatUnderLimit(acceptableHeatThreshold)) {
        indexer->setTargetRPM(indexerSpeed);
        unjamming_count = 0;
    }
    else {
        indexer->setTargetRPM(0.0f);
    }

    if (drivers->remote.keyPressed(Remote::Key::R)) {
        unjamming_count = 0;
    }
    



}

void AutoAgitatorIndexerCommand::end(bool) { feeder->setTargetRPM(0.0f); indexer->setTargetRPM(0.0f); }

bool AutoAgitatorIndexerCommand::isReady() {
    return true;
}

bool AutoAgitatorIndexerCommand::isFinished() const {
    return false;
}

}  // namespace src::Feeder