#include "full_auto_feeder_command.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers),
      feeder(feeder)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

bool commandRunningDisplay = false;

void FullAutoFeederCommand::initialize() {
    feeder->setTargetRPM(0);
    commandRunningDisplay = true;
}

void FullAutoFeederCommand::execute() {
    feeder->setTargetRPM(FEEDER_DEFAULT_RPM);
}

void FullAutoFeederCommand::end(bool) {
    feeder->setTargetRPM(0);
    commandRunningDisplay = false;
}

bool FullAutoFeederCommand::isReady() { return true; }

bool FullAutoFeederCommand::isFinished() const { return false; }

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE