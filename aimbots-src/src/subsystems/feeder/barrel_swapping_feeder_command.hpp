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

class BarrelSwappingFeederCommand : public TapCommand {
public:
    BarrelSwappingFeederCommand(
        src::Drivers*,
        FeederSubsystem*,
        src::Utils::RefereeHelperTurreted*,
        bool& barrelMovingFlag,
        float speed,
        float unjamSpeed,
        int UMJAM_TIMER_MS);
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

    bool& barrelMovingFlag;
    float speed;

    int UNJAM_TIMER_MS;

    MilliTimeout startupThreshold;
    MilliTimeout unjamTimer;
    float unjamSpeed = 0.0f;
};

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE