#pragma once

#include "subsystems/indexer/indexer.hpp"
#include "drivers.hpp"

#ifndef ENGINEER
namespace src::Indexer {
class StopIndexerCommand : public TapCommand {
public:
    StopIndexerCommand(src::Drivers*, IndexerSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "stop indexer"; }

private:
    src::Drivers* drivers;
    IndexerSubsystem* indexer;
};

}
#endif