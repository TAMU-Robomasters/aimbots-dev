#pragma once
#ifndef ENGINEER

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/feeder.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_helper.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Feeder {

class DualBarrelFeederCommand : public TapCommand {
public:
    DualBarrelFeederCommand(
        src::Drivers*,
        FeederSubsystem*,
        src::Utils::RefereeHelper*,
        bool& barrelMovingFlag,
        float speed = FEEDER_DEFAULT_RPM,
        float acceptableHeatThreshold = 0.90f,
        int UMJAM_TIMER_MS = 300);
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
    src::Utils::RefereeHelper* refHelper;

    bool& barrelMovingFlag;
    float speed;
    float acceptableHeatThreshold;

    int UNJAM_TIMER_MS;

    MilliTimeout startupThreshold;
    MilliTimeout unjamTimer;
    float unjamSpeed = 0.0f;
};

}  // namespace src::Feeder

#endif