#pragma once

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/feeder.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"



#include "feeder_limit_command.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder{
bool limitpressed = false;
FeederLimitCommand::FeederLimitCommand(
            src::Drivers* drivers, 
            FeederSubsystem* feeder,
            src::Utils::RefereeHelperTurreted* refHelper,
            float speed,
            float unjamSpeed,
            int UNJAM_TIMER_MS) : 
            drivers(drivers), 
            feeder(feeder), 
            refHelper(refHelper),
            speed(speed),
            isPressed(false), 
            UNJAM_TIMER_MS(UNJAM_TIMER_MS),
            unjamSpeed(-unjamSpeed)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    
}

void FeederLimitCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

void FeederLimitCommand::execute() {
    limitpressed = feeder->getPressed();
    if (limitpressed and (drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH)!= Remote::SwitchState::UP)) { //stops if switch gets pressed
        feeder->setTargetRPM(0.0f);
    }
    
    else if (drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH)== Remote::SwitchState::UP) {
        feeder->setTargetRPM(speed * 4);
        unjamTimer.restart(100);
     
    }

    else {
        feeder->setTargetRPM(speed);
    }
}
void FeederLimitCommand::end(bool) { feeder->setTargetRPM(0.0f); }

bool FeederLimitCommand::isReady() {
    return true;
}

bool FeederLimitCommand::isFinished() const {
    return false;
}

}
#endif 