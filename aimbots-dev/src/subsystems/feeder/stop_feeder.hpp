#pragma once

#include "drivers.hpp"
#include "subsystems/feeder/feeder.hpp"
#include "tap/control/command.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Feeder {

class StopFeeder : public TapCommand {
   public:
    StopFeeder(src::Drivers*, FeederSubsystem*);
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

}  // namespace src::Chassis