#pragma once

#include "subsystems/indexer/indexer.hpp"

#include "drivers.hpp"

#ifndef ENGINEER
namespace src::Indexer {
class BurstIndexerCommand : public TapCommand {
public:
    BurstIndexerCommand(
        src::Drivers*,
        IndexerSubsystem*,
        float speed = INDEXER_DEFAULT_RPM,
        float acceptableHeatThreshold = 0.90f,
        int burstLength = DEFAULT_BURST_LENGTH);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Burst Indexer Command"; }

    inline void setBurstLength(int newBurstLength) {
        startingTotalBallCount = indexer->getTotalLimitCount();
        burstLength = newBurstLength;
    }

private:
    src::Drivers* drivers;
    IndexerSubsystem* indexer;

    float speed;
    float acceptableHeatThreshold;
    bool canShoot;

    int startingTotalBallCount;
    int burstLength;
};

}  // namespace src::Indexer
#endif