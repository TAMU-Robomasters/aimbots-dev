#pragma once

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

class DualBarrelFeederCommand : public TapCommand {
public:
    DualBarrelFeederCommand(
        src::Drivers*,
        FeederSubsystem*,
        src::Utils::RefereeHelperTurreted*,
        std::array<BarrelID, 2> BARREL_IDS,
        uint8_t projectileBuffer,
        int UMJAM_TIMER_MS);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "run feeder"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    src::Utils::RefereeHelperTurreted* refHelper;

    std::array<BarrelID, 2> BARREL_IDS;

    int64_t antiOverheatEncoderThreshold;
    uint8_t projectileBuffer;

    int UNJAM_TIMER_MS;

    MilliTimeout startupThreshold;
    MilliTimeout unjamTimer;
};

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE