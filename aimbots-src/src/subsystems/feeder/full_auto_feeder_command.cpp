#include "full_auto_feeder_command.hpp"

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float speed, float acceptableHeatThreshold)
    : drivers(drivers),
      feeder(feeder),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      unjamSpeed(-speed / 2.0f) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FullAutoFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
}

void FullAutoFeederCommand::execute() {
    feeder->setTargetRPM(speed);
}

void FullAutoFeederCommand::end(bool) {
    feeder->setTargetRPM(0.0f);
}

bool FullAutoFeederCommand::isReady() {
    return feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

bool FullAutoFeederCommand::isFinished() const {
    return !feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

}  // namespace src::Feeder