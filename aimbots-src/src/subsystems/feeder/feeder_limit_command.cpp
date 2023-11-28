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
bool isAutoLoading = true;

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
    semiautoDelay.restart(0);
    limitswitchInactive.restart(0);
}

void FeederLimitCommand::execute() {
    
    /*
        11/18/2023
        Goal:
            - Have both an "auto feeder" and a "semi auto feeder" states
                - auto feeder == automatically fire until the limit switch clicks
                - semi auto feeder == fire one projectile
        

        Currently:
            - Works as it should until we enter semi auto
            - Upon going to semi auto, the system shuts off completely lmao lol us too :)
 
        ^^^ double check the goal w jack if this is correct ^^^
            if not, update the goal & the corresponding date :P
        
        we need to fix :D
            - dimitri & nico
            
    */


    // Updates the limit switch state (is pressed or not)
    limitpressed = feeder->getPressed(); 
    // Updates the previous controller switch state (is up or not)
    isPrevSemiauto = isCurrSemiauto;
    // Updates the current controller switch state
    isCurrSemiauto = drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP; 

    
    // Checks if the limit switch is pressed & is "not killed" (refer to )
    if (limitpressed and limitswitchInactive.isExpired()){
        isAutoLoading = false;
    }
    // if the fire mode has been activated (with timer semiauto) then check if your still in the other timers range to keep shooting, else set the feeder to 0 until reset timers
    if(semiautoDelay.isExpired()){
        if ( isAutoLoading )
            feeder->setTargetRPM(speed);
        else
            feeder->setTargetRPM(0.0f);
    }

    // Checks if the controller is in its semiautomatic state
    //      i.e. controller switch goes from mid to up
    // If so, it (should) launch the currently loaded ball and turn off until the controller exits semi auto (ie cSwitch goes to mid)
    //if (isCurrSemiauto and !isPrevSemiauto){
    if (drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH)== Remote::SwitchState::UP){
 //   if (true){
        feeder->setTargetRPM(speed * 2);
        semiautoDelay.restart(1000);
        isAutoLoading = true;
        limitswitchInactive.restart(4000);
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