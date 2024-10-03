#pragma once
#include "tap/control/command.hpp"
#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

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

#endif  // #ifdef FEEDER_COMPATIBLE