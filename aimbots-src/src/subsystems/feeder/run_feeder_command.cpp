#include "run_feeder_command.hpp"

namespace src::Feeder {
RunFeederCommand::RunFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers), feeder(feeder), speed(0) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void RunFeederCommand::initialize() {
    speed = 0.0f;
    feeder->setTargetRPM(0.0f);
}

void RunFeederCommand::execute() {
    speed = FEEDER_DEFAULT_RPM;
    feeder->setTargetRPM(speed);
}

void RunFeederCommand::end(bool) {}

bool RunFeederCommand::isReady() {
    return true;
}

bool RunFeederCommand::isFinished() const {
    return false;  // finished condition (button released) or their api is nice and we don't have to
}
}  // namespace src::Feeder