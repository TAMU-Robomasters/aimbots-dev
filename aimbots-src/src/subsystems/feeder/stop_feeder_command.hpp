#pragma once

#ifndef ENGINEER
#include "tap/control/command.hpp"

#include "subsystems/feeder/feeder.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Feeder {

class StopFeederCommand : public TapCommand {
public:
    StopFeederCommand(src::Drivers*, FeederSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "stop feeder"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
};

}  // namespace src::Feeder

#endif