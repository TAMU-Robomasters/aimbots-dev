#pragma once

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

enum class FeederState : uint8_t {
    initialize = 0,
    priming,
    primed,
    firing,
    jammed, // how do i tell if it's jammed?
    undefined
};

class FeederShotTimingCommand : public TapCommand {
public:
    FeederShotTimingCommand(
        src::Drivers* drivers,
        FeederSubsystem* feeder,
        src::Utils::RefereeHelperTurreted*,
        int UMJAM_TIMER_MS = 300);
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

    MilliTimeout startupThreshold;
    MilliTimeout unjamTimer;
    // MilliTimeout semiautoDelay;
    MilliTimeout limitswitchInactive;

    FeederState currentFeederState;

    bool currFireState = false;
    bool prevFireState = false;

    bool currSwitchState = false;
    bool prevSwitchState = false;
};
}  // namespace src::Feeder

#endif