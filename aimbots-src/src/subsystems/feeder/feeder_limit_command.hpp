#pragma once

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/feeder.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder{
    
class FeederLimitCommand : public TapCommand {
    public:
        FeederLimitCommand(src::Drivers* drivers, FeederSubsystem* feeder);
        void initialize() override;

        void execute() override;
        void end(bool interrupted) override;
        bool isReady() override;

        bool isFinished() const override;

        const char* getName() const override { return "limit feeder"; }
        


    private:
        src::Drivers* drivers;
        FeederSubsystem* feeder;
        bool isPressed; // might be deletable but we keeping it just in case ig :shrug:
        float rpm;//你妈胖
        


};
}


#endif