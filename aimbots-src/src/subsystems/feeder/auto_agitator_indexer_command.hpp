#pragma once

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/feeder.hpp"
#include "subsystems/indexer/indexer.hpp"

#include "utils/common_types.hpp"
#include "utils/ref_helper.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Feeder {

class AutoAgitatorIndexerCommand : public TapCommand {
public:
    AutoAgitatorIndexerCommand(
        src::Drivers*,
        FeederSubsystem*,
        src::Indexer::IndexerSubsystem*,
        src::Utils::RefereeHelper*,
        float feederSpeed,
        float indexerSpeed,
        float acceptableHeatThreshold = 0.90f,
        int UNJAM_TIMER_MS = 300,
        int MAX_UNJAM_COUNT = 3);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    void setSpeed(float speed) { this->feederSpeed = speed; }

    const char* getName() const override { return "run agitator and indexer"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    src::Indexer::IndexerSubsystem* indexer;
    
    src::Utils::RefereeHelper* refHelper;

    float feederSpeed;
    float indexerSpeed;
    float acceptableHeatThreshold;
    int unjamming_count = 0;

    int UNJAM_TIMER_MS;
    int MAX_UNJAM_COUNT;

    MilliTimeout startupThreshold;
    MilliTimeout unjamTimer;
    float unjamSpeed = 0.0f;
};

}  // namespace src::Feeder