#include "full_auto_feeder_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "subsystems/feeder/control/feeder.hpp"

#ifdef FEEDER_COMPATIBLE

bool currentTimer = 0;

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder   
) : drivers(drivers),
    feeder(feeder)
  {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
  }  

void FullAutoFeederCommand::initialize() {
    jamTimer.execute();
    reverseTimer.execute();
    feeder->setTargetRPM(0, 4000);
    jamTimer.restart(500);
}

void FullAutoFeederCommand::execute() {
    
    if(reverseTimer.isExpired()){
        feeder->setTargetRPM(0, 4000);
    }

    if(jamTimer.isExpired()){
        
        if (abs(feeder->getCurrentRPM(0)) < 20){

            feeder->setTargetRPM(0, -1000);
            reverseTimer.restart(1000);
            jamTimer.restart(500);
        }
    
    }
}

void FullAutoFeederCommand::end(bool interrupted){
}

bool FullAutoFeederCommand::isReady() {
    return true;
}

bool FullAutoFeederCommand::isFinished() const{
    return false;
}
    
}

#endif // #ifdef FEEDER_COMPATIBLE