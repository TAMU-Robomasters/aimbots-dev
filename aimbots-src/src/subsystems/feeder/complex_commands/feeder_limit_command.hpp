#pragma once

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

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

    bool isFinished() const override;

    void execute() override;
    void end(bool interrupted) override;

    const char* getName() const override { return "limit feeder"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    src::Utils::RefereeHelperTurreted* refHelper;
    int UNJAM_TIMER_MS;
    bool canShoot = false;

    /*Initialize any functions here*/
    //void exampleFunc();

    MilliTimeout timer1;
    MilliTimeout timer2;
    MilliTimeout timer3;
    MilliTimeout timer4;
    MilliTimeout timer5;
    MilliTimeout timer6;

    /*initialize any enums here*/
    // enum example{
    //     ex1,
    //     ex2,
    //     ex3
    // };
    uint32_t initialTime;
};
}  // namespace src::Feeder

#endif