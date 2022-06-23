#pragma once

#include "drivers.hpp"
#include "subsystems/feeder/feeder.hpp"
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Feeder {

class BurstFeederCommand : public TapCommand {
   public:
    BurstFeederCommand(src::Drivers*, FeederSubsystem*, int burstLength);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Burst Feeder Command"; }

    inline void setBurstLength(int newBurstLength) {
        initialTotalBallCount = feeder->getTotalLimitCount();
        burstLength = newBurstLength;
    }

   private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    float speed;

    int burstLength;
    int initialTotalBallCount;
};

}  // namespace src::Feeder