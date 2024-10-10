#include "stop_feeder_command.hpp"
#include "full_auto_feeder_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "subsystems/feeder/control/feeder.hpp"

#ifdef FEEDER_COMPATIBLE


namespace src::Feeder {

StopFeederCommand::StopFeederCommand(
    src::Drivers *drivers,
    FeederSubsystem *feeder
) : drivers(drivers),
    feeder(feeder)
  {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
  }  

void StopFeederCommand::initialize(){
    
    feeder->setTargetRPM(0, 0);
}

void StopFeederCommand::execute() {
    
}

void StopFeederCommand::end(bool interrupted){
}

bool StopFeederCommand::isReady() {
    return true;
}

bool StopFeederCommand::isFinished() const{
    return false;
}
}

#endif // #ifdef FEEDER_COMPATIBLE