#include "run_feeder_command.hpp"
namespace src::Feeder {

RunFeederCommand::RunFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float speed, float acceptableHeatThreshold)
    : drivers(drivers),
      feeder(feeder),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void RunFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
}

void RunFeederCommand::execute() {
    feeder->setTargetRPM(speed);
}

void RunFeederCommand::end(bool) {}

bool RunFeederCommand::isReady() {
    return feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

bool RunFeederCommand::isFinished() const {
    return !feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

}  // namespace src::Feeder