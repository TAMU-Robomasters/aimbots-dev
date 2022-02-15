#pragma once

#include "drivers.hpp"
#include "subsystems/feeder/feeder.hpp"
#include "tap/control/command.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Feeder {

class RunFeederCommand : public TapCommand {
   public:
    RunFeederCommand(src::Drivers*, FeederSubsystem*, float);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "run feeder"; }

   private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    float speed;
};

}  // namespace src::Chassis