#pragma once
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/feeder.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {
class BurstFeederCommand : public TapCommand {
public:
    BurstFeederCommand(
        src::Drivers*,
        FeederSubsystem*,
        src::Utils::RefereeHelperTurreted*,
        float speed = FEEDER_DEFAULT_RPM,
        float acceptableHeatThreshold = 0.90f,
        int burstLength = DEFAULT_BURST_LENGTH);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Burst Feeder Command"; }

    inline void setBurstLength(int newBurstLength) {
        startingTotalBallCount = feeder->getTotalLimitCount();
        burstLength = newBurstLength;
    }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    src::Utils::RefereeHelperTurreted* refHelper;

    float speed;
    float acceptableHeatThreshold;
    bool canShoot;

    int startingTotalBallCount;
    int burstLength;
};

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE