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
int limitSwitchDownTime;
int feederFiringRPM;

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
    feeder->setTargetRPM(0.0f,0);
    feeder->setTargetRPM(0.0f,1);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    limitswitchInactive.restart(0);
}

void FeederLimitCommand::execute() {
    
    /*
        12/04/2023
        Goal:
            - Implement a semi-auto functionality for hero robot
            - Switch states
                - low: have feeder active until limit switch is pressed
                - mid: same as low but have shooter wheels active
                - up: fires one projectile

        Currently:
            - It works!
        
        we are done :D
            - dimitri, lohit, josh, yajat, luis
            
    */

    // Updates the limit switch state (is pressed or not)
    limitpressed = feeder->getPressed(); 
    // Updates the previous controller switch state (is up or not)
    isPrevSemiauto = isCurrSemiauto;
    // Updates the current controller switch state
    isCurrSemiauto = drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP; 
    // States how long the limit switch is ignored when firing a projectile
    limitSwitchDownTime = 350;
    // States the speed of the feeder wheel when firing
    feederFiringRPM = speed * 2;
    

    
    // Checks if the limit switch is pressed & is "not killed" (refer to )
    if (limitswitchInactive.isExpired()){
        isFiring = false;
    }
    // if the fire mode has been activated (with timer semiauto) then check if your still in the other timers range to keep shooting, else set the feeder to 0 until reset timers
    if(!isFiring){
        feeder->setTargetRPM(0.0f,1);
        if (!limitpressed){
            feeder->setTargetRPM(speed,0);
            }
        else{
            feeder->setTargetRPM(0.0f,0);
        }
    }
    
    // Checks if the controller is in its semiautomatic state
    //      i.e. controller switch goes from mid to up
    // If so, it (should) launch the currently loaded ball and turn off until the controller exits semi auto (ie cSwitch goes to mid)
    if (isCurrSemiauto and !isPrevSemiauto){
        feeder->setTargetRPM(feederFiringRPM, 1);
        isFiring = true;
        limitswitchInactive.restart(limitSwitchDownTime);
    }

    

    
}
void FeederLimitCommand::end(bool) {
     feeder->setTargetRPM(0.0f,0); 
     feeder->setTargetRPM(0.0f,1);
     }

bool FeederLimitCommand::isReady() {
    return true;
}

bool FeederLimitCommand::isFinished() const {
    return false;
}

}
#endif 