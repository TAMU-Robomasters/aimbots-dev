#include "full_auto_feeder_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "subsystems/feeder/control/feeder.hpp"

#ifdef FEEDER_COMPATIBLE


namespace src::Feeder {

//debug variables
    bool isRunningDisplay = false;


FullAutoFeederCommand::FullAutoFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers),
      feeder(feeder)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FullAutoFeederCommand::initialize() {
    isRunningDisplay = true;
    feeder->setTargetRPM(4000);
}

void FullAutoFeederCommand::execute() {
    //feeder->setTargetRPM(4000);
}

void FullAutoFeederCommand::end(bool) {
    isRunningDisplay = false;
}

bool FullAutoFeederCommand::isReady() { return true; }

bool FullAutoFeederCommand::isFinished() const { return false; }

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE