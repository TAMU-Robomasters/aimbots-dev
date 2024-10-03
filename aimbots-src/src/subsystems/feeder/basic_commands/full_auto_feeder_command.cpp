#include "full_auto_feeder_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "subsystems/feeder/control/feeder.hpp"

#ifdef FEEDER_COMPATIBLE


namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(
    src::Drivers *drivers,
    FeederSubsystem *feeder   
) : drivers(drivers),
    feeder(feeder)
  {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
  }  

void FullAutoFeederCommand::initialize() {
    feeder->initialize();
    feeder->setTargetRPM(FEEDER_MOTOR_IDS[0], 300);
}

void FullAutoFeederCommand::execute() {
    feeder->refresh();
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