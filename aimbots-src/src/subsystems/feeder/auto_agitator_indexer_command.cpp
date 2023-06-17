#include "auto_agitator_indexer_command.hpp"

using namespace src::Feeder;
using namespace src::Indexer;

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
    : TapComprisedCommand(drivers),
      feeder(feeder),
      indexer(indexer),
      refHelper(refHelper),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      MAX_UNJAM_COUNT(MAX_UNJAM_COUNT),
      runFeederCommand(drivers, feeder, refHelper, feederSpeed, 1.0f, UNJAM_TIMER_MS),
      stopFeederCommand(drivers,feeder),
      runIndexerCommand(drivers,indexer,indexerSpeed, acceptableHeatThreshold),
      stopIndexerCommand(drivers,indexer) 
{
    this->comprisedCommandScheduler.registerSubsystem(feeder);
    this->comprisedCommandScheduler.registerSubsystem(indexer);
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(indexer));
}

void AutoAgitatorIndexerCommand::initialize() {
    if (comprisedCommandScheduler.isCommandScheduled(&runIndexerCommand)) comprisedCommandScheduler.removeCommand(&runIndexerCommand, true);
    if (!comprisedCommandScheduler.isCommandScheduled(&runFeederCommand)) comprisedCommandScheduler.addCommand(&runFeederCommand);

    startupTimeout.restart(500);
}

//The plan
//Run this either as a default command, or paired to a remote switch that should be "always on"
//Read from an additional remote input within this command, as the "shoot" trigger

//While command is running, run agitator at full power
//After reaching an unjam, count it.
//After attempting unjam 3 times, if problem persists, halt (assume fully loaded)
//Read a different remote input to reset the unjam counter (force reload, essentially)
//Once the "shoot" trigger is pressed, start moving the indexer, and reset the unjam counter


void AutoAgitatorIndexerCommand::execute() {

    if (drivers->remote.getMouseL()) {
        scheduleIfNotScheduled(this->comprisedCommandScheduler,&runIndexerCommand);
        unjamming_count = 0;
        fullyLoaded = false;
    }
    else {
        scheduleIfNotScheduled(this->comprisedCommandScheduler,&stopIndexerCommand);
    }

    if (abs(feeder->getCurrentRPM()) <= 10.0f && !jamDetected && startupTimeout.execute()) {
        jamDetected = true;
        unjamming_count++;
    }

    if (unjamming_count >= MAX_UNJAM_COUNT && !fullyLoaded) {
        fullyLoaded = true;
    }

    if (drivers->remote.keyPressed(Remote::Key::R)) {
        unjamming_count = 0;
        fullyLoaded = false;
    }

    if (fullyLoaded) {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &stopFeederCommand);
    }
    else {
        scheduleIfNotScheduled(this->comprisedCommandScheduler,&runFeederCommand);
    }
    
    comprisedCommandScheduler.run();
}

void AutoAgitatorIndexerCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &runIndexerCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &runFeederCommand, interrupted);
    feeder->setTargetRPM(0.0f); 
    indexer->setTargetRPM(0.0f); }

bool AutoAgitatorIndexerCommand::isReady() {
    return true;
}

bool AutoAgitatorIndexerCommand::isFinished() const {
    return false;
}
