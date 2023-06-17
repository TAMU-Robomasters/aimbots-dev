#pragma once

#include "subsystems/feeder/feeder.hpp"
#include "subsystems/feeder/full_auto_feeder_command.hpp"
#include "subsystems/feeder/stop_feeder_command.hpp"

#include "subsystems/indexer/indexer.hpp"
#include "subsystems/indexer/full_auto_indexer_command.hpp"
#include "subsystems/indexer/stop_indexer_command.hpp"

#include "utils/common_types.hpp"
#include "utils/ref_helper.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

using namespace src::Feeder;
using namespace src::Indexer;

class AutoAgitatorIndexerCommand : public TapComprisedCommand {
public:
    AutoAgitatorIndexerCommand(
        src::Drivers*,
        FeederSubsystem*,
        IndexerSubsystem*,
        src::Utils::RefereeHelper*,
        float feederSpeed,
        float indexerSpeed,
        float acceptableHeatThreshold = 0.80f,
        int UNJAM_TIMER_MS = 300,
        int MAX_UNJAM_COUNT = 3);

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "run agitator and indexer comprised command"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    IndexerSubsystem* indexer;
    
    src::Utils::RefereeHelper* refHelper;

    int UNJAM_TIMER_MS;
    int MAX_UNJAM_COUNT;

    bool jamDetected = false;
    bool fullyLoaded = false;

    FullAutoFeederCommand runFeederCommand;
    StopFeederCommand stopFeederCommand;

    FullAutoIndexerCommand runIndexerCommand;
    StopIndexerCommand stopIndexerCommand;

    MilliTimeout startupTimeout;

    int unjamming_count = 0;

};