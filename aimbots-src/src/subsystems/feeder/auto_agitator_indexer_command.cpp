#include "auto_agitator_indexer_command.hpp"

#ifdef FEEDER_COMPATIBLE

using namespace src::Feeder;
using namespace src::Indexer;

AutoAgitatorIndexerCommand::AutoAgitatorIndexerCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Indexer::IndexerSubsystem* indexer,
    src::Utils::RefereeHelperTurreted* refHelper,
    float feederSpeed,
    float indexerSpeed,
    float acceptableHeatThreshold,
    int UNJAM_TIMER_MS,
    int MAX_UNJAM_COUNT)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      feeder(feeder),
      indexer(indexer),
      refHelper(refHelper),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      MAX_UNJAM_COUNT(MAX_UNJAM_COUNT),
      runFeederCommand(drivers, feeder, refHelper, feederSpeed, 1500.0f, 1, UNJAM_TIMER_MS),
      stopFeederCommand(drivers, feeder),
      runIndexerCommand(drivers, indexer, refHelper, indexerSpeed, acceptableHeatThreshold),
      /*reverseIndexerCommand(drivers, indexer, refHelper, -indexerSpeed * 0.5, 1.0f),*/
      stopIndexerCommand(drivers, indexer) {
    this->comprisedCommandScheduler.registerSubsystem(feeder);
    this->comprisedCommandScheduler.registerSubsystem(indexer);
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(indexer));
}

// DEBUG VARIABLES --------------------------------

bool commandIsRunning = false;
bool unjamDisplay = false;
bool isLoadedDisplay = false;
int unjamCountDisplay = 0;

//------------------------------------------------

void AutoAgitatorIndexerCommand::initialize() {
    if (comprisedCommandScheduler.isCommandScheduled(&runIndexerCommand))
        comprisedCommandScheduler.removeCommand(&runIndexerCommand, true);
    if (!comprisedCommandScheduler.isCommandScheduled(&runFeederCommand))
        comprisedCommandScheduler.addCommand(&runFeederCommand);

    unjamTimer.restart(0);
    startupTimeout.restart(500);
    reverseTimer.restart(0);
}

// The plan
// Run this either as a default command, or paired to a remote switch that should be "always on"
// Read from an additional remote input within this command, as the "shoot" trigger

// While command is running, run agitator at full power
// After reaching an unjam, count it.
// After attempting unjam 3 times, if problem persists, halt (assume fully loaded)
// Read a different remote input to reset the unjam counter (force reload, essentially)
// Once the "shoot" trigger is pressed, start moving the indexer, and reset the unjam counter

// uint16_t currHeat = 69;
// uint16_t currHeatLimit = 420;
// uint16_t chassisPowerLimit = 77;
void AutoAgitatorIndexerCommand::execute() {
    // currHeat = refHelper->getCurrBarrelHeat();
    // currHeatLimit = refHelper->getCurrBarrelLimit();
    // chassisPowerLimit = drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;

    commandIsRunning = true;
    unjamDisplay = jamDetected;
    isLoadedDisplay = fullyLoaded;
    unjamCountDisplay = unjamming_count;

    if (drivers->remote.getMouseL() || drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP) {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &runIndexerCommand);
        unjamming_count = 0;
        fullyLoaded = false;
        // reverseTimer.restart(100);
    } else {
        // This might actually break the feeder, don't uncomment for now
        /*if (reverseTimer.isExpired()) {
            scheduleIfNotScheduled(this->comprisedCommandScheduler,&stopIndexerCommand);
        }
        else {
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &reverseIndexerCommand);
        }*/
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &stopIndexerCommand);
    }

    if (unjamTimer.execute()) {
        jamDetected = false;
        unjamming_count++;
        startupTimeout.restart(500);
    }

    if (abs(feeder->getCurrentRPM()) <= 10.0f && !jamDetected && startupTimeout.isExpired()) {
        jamDetected = true;
        unjamTimer.restart(UNJAM_TIMER_MS);
    }

    if (unjamming_count >= MAX_UNJAM_COUNT && !fullyLoaded) {
        fullyLoaded = true;
    }

    if (drivers->remote.keyPressed(Remote::Key::B)) {
        unjamming_count = 0;
        fullyLoaded = false;
    }

    if (fullyLoaded) {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &stopFeederCommand);
    } else {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &runFeederCommand);
    }

    comprisedCommandScheduler.run();
}

void AutoAgitatorIndexerCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &runIndexerCommand, interrupted);
    // descheduleIfScheduled(this->comprisedCommandScheduler, &reverseIndexerCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &runFeederCommand, interrupted);
    feeder->setTargetRPM(0.0f);
    indexer->setTargetRPM(0.0f);
    unjamming_count = 0;
    commandIsRunning = false;
}

bool AutoAgitatorIndexerCommand::isReady() { return true; }

bool AutoAgitatorIndexerCommand::isFinished() const { return false; }

#endif  // #ifdef FEEDER_COMPATIBLE
