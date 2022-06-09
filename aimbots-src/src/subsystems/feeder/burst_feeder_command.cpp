#include "burst_feeder_command.hpp"

namespace src::Feeder {
BurstFeederCommand::BurstFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers), feeder(feeder), speed(0) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void BurstFeederCommand::initialize() {
    feeder->setBurstLength(4);

    speed = FEEDER_DEFAULT_RPM;
    feeder->setTargetRPM(speed);

    initialTotalBallCount = feeder->getTotalLimitCount();
}

void BurstFeederCommand::execute() {
    speed = FEEDER_DEFAULT_RPM;
    feeder->setTargetRPM(speed);
}

void BurstFeederCommand::end(bool) {
    feeder->setTargetRPM(0.0);
}

bool BurstFeederCommand::isReady() {
    return true;
}

bool BurstFeederCommand::isFinished() const {
    int elapsedTotal = feeder->getTotalLimitCount() - initialTotalBallCount;

    return elapsedTotal >= feeder->getBurstLength();
}
}  // namespace src::Feeder