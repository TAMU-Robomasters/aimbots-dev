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

class FeederLimitCommand : public TapCommand {
public:
    FeederLimitCommand(
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
};
}  // namespace src::Feeder

#endif