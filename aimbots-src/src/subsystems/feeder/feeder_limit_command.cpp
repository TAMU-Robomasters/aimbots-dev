#pragma once

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/feeder.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"



#include "feeder_limit_command.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder{

FeederLimitCommand::FeederLimitCommand(src::Drivers* drivers, FeederSubsystem* feeder) : 
            drivers(drivers), 
            feeder(feeder), 
            isPressed(false), 
            rpm(1000.0f) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    
}

void FeederLimitCommand::initialize() {
    
}

void FeederLimitCommand::execute() {
    if (feeder->getPressed()) {
        feeder->setTargetRPM(0);
    }

    else {
        feeder->setTargetRPM(rpm);
    }
    
}


}
#endif 