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

class FullAutoFeederCommand : public TapCommand {
public:
    FullAutoFeederCommand(
        src::Drivers*,
        FeederSubsystem*,
        src::Utils::RefereeHelperTurreted*,
        float speed,
        float unjamSpeed,
        int projectileBuffer = 0,
        int UNJAM_TIMER_MS = 300);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    void setSpeed(float speed) { this->speed = speed; }

    const char* getName() const override { return "run feeder"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    src::Utils::RefereeHelperTurreted* refHelper;

    int64_t antiOverheatEncoderThreshold;

    float speed;

    int projectileBuffer;

    int UNJAM_TIMER_MS;

    MilliTimeout startupThreshold;
    MilliTimeout unjamTimer;
    float unjamSpeed = 0.0f;
};

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE