#pragma once

#include "subsystems/feeder/feeder.hpp"
#include "subsystems/feeder/full_auto_feeder_command.hpp"
#include "subsystems/feeder/stop_feeder_command.hpp"
#include "subsystems/indexer/full_auto_indexer_command.hpp"
#include "subsystems/indexer/indexer.hpp"
#include "subsystems/indexer/stop_indexer_command.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

using namespace src::Feeder;
using namespace src::Indexer;

class AutoAgitatorIndexerCommand : public TapComprisedCommand {
public:
    AutoAgitatorIndexerCommand(
        src::Drivers*,
        FeederSubsystem*,
        IndexerSubsystem*,
        src::Utils::RefereeHelperTurreted*,
        float feederSpeed,
        float indexerSpeed,
        float acceptableHeatThreshold,
        int UNJAM_TIMER_MS,
        int MAX_UNJAM_COUNT);

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

    src::Utils::RefereeHelperTurreted* refHelper;

    int UNJAM_TIMER_MS;
    int MAX_UNJAM_COUNT;

    bool jamDetected = false;
    bool fullyLoaded = false;

    FullAutoFeederCommand runFeederCommand;
    StopFeederCommand stopFeederCommand;

    FullAutoIndexerCommand runIndexerCommand;
    //FullAutoIndexerCommand reverseIndexerCommand;
    StopIndexerCommand stopIndexerCommand;

    MilliTimeout startupTimeout;
    MilliTimeout unjamTimer;
    MilliTimeout reverseTimer;

    int unjamming_count = 0;
};

#endif  // #ifdef FEEDER_COMPATIBLE