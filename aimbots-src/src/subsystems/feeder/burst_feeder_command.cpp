#include "burst_feeder_command.hpp"

namespace src::Feeder {
BurstFeederCommand::BurstFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers), feeder(feeder), speed(0) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void BurstFeederCommand::initialize() {
    speed = FEEDER_DEFAULT_SPEED;
    feeder->setTargetRPM(speed);
    
    leftLimitCount = feeder->getLeftLimitCount();
    rightLimitCount = feeder->getRightLimitCount();
    totalLimitCount = feeder->getTotalLimitCount();
}

void BurstFeederCommand::execute() {
}

void BurstFeederCommand::end(bool) {
    feeder->setTargetRPM(0.0);
}

bool BurstFeederCommand::isReady() {
}

bool BurstFeederCommand::isFinished() const {
    int elapsedLeft = feeder->getLeftLimitCount() - leftLimitCount;
    int elapsedRight = feeder->getRightLimitCount() - rightLimitCount;
    int elapsedTotal = feeder->getTotalLimitCount() - totalLimitCount;

    return elapsedTotal > 10;
}
}  // namespace src::Feeder