#include "run_feeder_command.hpp"

namespace src::Feeder {

RunFeederCommand::RunFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float speed, float acceptableHeatThreshold)
    : drivers(drivers),
      feeder(feeder),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      canShoot(true) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void RunFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
}

void RunFeederCommand::execute() {
    canShoot = feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);

    feeder->setTargetRPM(speed);
}

void RunFeederCommand::end(bool) {}

bool RunFeederCommand::isReady() {
    return true;
}

bool RunFeederCommand::isFinished() const {
    return !canShoot;
}

}  // namespace src::Feeder