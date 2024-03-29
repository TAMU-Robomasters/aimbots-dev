#pragma once

#include "subsystems/indexer/indexer.hpp"

#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

namespace src::Indexer {
class FullAutoIndexerCommand : public TapCommand {
public:
    FullAutoIndexerCommand(src::Drivers*, IndexerSubsystem*, src::Utils::RefereeHelperTurreted*, float speed, float acceptableHeatThreshold = 0.90f);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    void setSpeed(float speed) { this->speed = speed; }

    const char* getName() const override { return "run indexer"; }

private:
    src::Drivers* drivers;
    IndexerSubsystem* indexer;
    src::Utils::RefereeHelperTurreted* refHelper;

    float speed;
    float acceptableHeatThreshold;

    MilliTimeout startupThreshold;
    MilliTimeout unjamTimer;
    float unjamSpeed = 0.0f;
};
}  // namespace src::Indexer