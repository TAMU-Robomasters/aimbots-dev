#include "full_auto_feeder_command.hpp"

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers),
      feeder(feeder),
      speed(0)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FullAutoFeederCommand::initialize() {
    speed = 0.0f;
    feeder->setTargetRPM(speed);
}

void FullAutoFeederCommand::execute() {
    speed = FEEDER_DEFAULT_RPM;
    feeder->setTargetRPM(speed);
}

void FullAutoFeederCommand::end(bool) {}

bool FullAutoFeederCommand::isReady() {
    return true;
}

bool FullAutoFeederCommand::isFinished() const {
    return false;  // finished condition (button released) or their api is nice and we don't have to
}
}  // namespace src::Feeder