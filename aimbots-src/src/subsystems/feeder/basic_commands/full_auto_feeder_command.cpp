#include "full_auto_feeder_command.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers),
      feeder(feeder)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FullAutoFeederCommand::initialize() {}

void FullAutoFeederCommand::execute() {}

void FullAutoFeederCommand::end(bool) {}

bool FullAutoFeederCommand::isReady() { return true; }

bool FullAutoFeederCommand::isFinished() const { return false; }

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE