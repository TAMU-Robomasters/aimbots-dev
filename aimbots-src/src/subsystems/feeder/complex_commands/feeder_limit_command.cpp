#include "feeder_limit_command.hpp"

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

/*
Define any variables you need here, we have provided some here to give you sensor data and other useful things
*/
bool limitPressed = false;
bool wantToShoot = false;
float currRPM = 0.0;


//subsystem declaration stuff, do not touch
FeederLimitCommand::FeederLimitCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FeederLimitCommand::initialize() {
    //makes sure robot doesnt shoot when turned on
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    //to activate motors call feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::activateFeederMotor);
    //and deactivate at the end if your turning them off. to call a specific motor replace ALL with
    //LOADER or KICKER, these are described in the video

    //initialized 6 timers, you can use as many or as few as you like.
    //units for the timer are in milliseconds, call .restart(milliseconds) to activate the timer for that long
    //check if a timer has expired by calling timer.isExpired(), will return true if timer is not running/is done
    timer1.restart(0);
    timer2.restart(0);
    timer3.restart(0);
    timer4.restart(0);
    timer5.restart(0);
    timer6.restart(0);

    //sys time on initialization, might be useful for heat managment
    //can call getTimeMilliseconds() anywhere to get current system
    initialTime = tap::arch::clock::getTimeMilliseconds();
}

void FeederLimitCommand::execute() {
    /*
    Update calls to get sensor data and input, dont touch but do use these variables
    to check input states and sensor data
    */
    //updates if the limit switch detects a ball, true = ball detected, false means no ball detected
    limitPressed = feeder->getPressed();
    //gets if the driver wants to shoot a ball, will hold true for a little bit before dropping back down to false
    wantToShoot = (drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP || drivers->remote.getMouseL()==true || drivers->cvCommunicator.shouldFire());
    //gets current rpm of the base feeder motor, might be useful for unjam
    currRPM = feeder->getCurrentRPM(0);

    //Do stuff
  
}

/*declare any functions down here(make sure to include in hpp)*/

//void exampleFunc(){
//  do stuff
//};



//dont worry about this stuff
void FeederLimitCommand::end(bool) { feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor); }

bool FeederLimitCommand::isFinished() const { return false; }

}  
#endif
