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
        int64_t SINGLE_SHOT_ENCODER_TICKS =
            (DJIMotor::ENC_RESOLUTION * FEEDER_GEAR_RATIOS[0]) / PROJECTILES_PER_FEEDER_ROTATION);
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
    int64_t SINGLE_SHOT_ENCODER_TICKS;
    int64_t singleShotStartEncoder;
    int singleShotDirectionSign;

    MilliTimeout startupThreshold;
    MilliTimeout unjamTimer;
    // MilliTimeout semiautoDelay;
    MilliTimeout limitswitchInactive;
};
}  // namespace src::Feeder

#endif
#endif
