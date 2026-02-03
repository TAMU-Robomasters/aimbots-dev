#pragma once

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE
#ifdef NO_LIMIT_COMPATIBLE

namespace src::Feeder {

class EncoderSingleShotCommand : public TapCommand {
public:
    EncoderSingleShotCommand(
        src::Drivers* drivers,
        FeederSubsystem* feeder,
        src::Utils::RefereeHelperTurreted*,
        int UNJAM_TIMER_MS = 300,
        int SINGLE_SHOT_MS = 500);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "limit feeder"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    src::Utils::RefereeHelperTurreted* refHelper;
    int UNJAM_TIMER_MS;
    int SINGLE_SHOT_MS;

    MilliTimeout startupThreshold;
    MilliTimeout unjamTimer;
    MilliTimeout singleShotTimer;
    // MilliTimeout semiautoDelay;
    MilliTimeout limitswitchInactive;
};
}  // namespace src::Feeder

#endif
#endif
