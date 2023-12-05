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
bool isCurrSemiauto = false;
bool isPrevSemiauto = false;
bool isFiring = false;

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
    limitswitchInactive.restart(0);
}

void FeederLimitCommand::execute() {
    
    /*
        11/29/2023
        Goal:
            - Implement a semi-auto functionality for hero robot
            - Switch states
                - low: have feeder active until limit switch is pressed
                - mid: same as low but have shooter wheels active
                - up: fires one projectile
        

        Currently:
            - We think it works! we just need to test with actual projectiles to confirm
            - Currently *not* in tune, will be done later
        
        we need to test :D
            - dimitri, lohit, josh, yajat
            
    */

    // Updates the limit switch state (is pressed or not)
    limitpressed = feeder->getPressed(); 
    // Updates the previous controller switch state (is up or not)
    isPrevSemiauto = isCurrSemiauto;
    // Updates the current controller switch state
    isCurrSemiauto = drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP; 
    

    
    // Checks if the limit switch is pressed & is "not killed" (refer to )
    if (limitswitchInactive.isExpired()){
        isFiring = false;
    }
    // if the fire mode has been activated (with timer semiauto) then check if your still in the other timers range to keep shooting, else set the feeder to 0 until reset timers
    if(!isFiring){
        if (!limitpressed)
            feeder->setTargetRPM(speed);
        else
            feeder->setTargetRPM(0.0f);
    }
    
    // Checks if the controller is in its semiautomatic state
    //      i.e. controller switch goes from mid to up
    // If so, it (should) launch the currently loaded ball and turn off until the controller exits semi auto (ie cSwitch goes to mid)
    if (isCurrSemiauto and !isPrevSemiauto){
        feeder->setTargetRPM(speed * 2);
        isFiring = true;
        limitswitchInactive.restart(350);
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