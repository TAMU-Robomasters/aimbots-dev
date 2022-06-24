#include "burst_feeder_command.hpp"

namespace src::Feeder {

BurstFeederCommand::BurstFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, int burstLength)
    : drivers(drivers),
      feeder(feeder),
      speed(0),
      burstLength(burstLength)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void BurstFeederCommand::initialize() {
    initialTotalBallCount = feeder->getTotalLimitCount();

    speed = FEEDER_DEFAULT_RPM;
    feeder->setTargetRPM(speed);
}

void BurstFeederCommand::execute() {
    speed = FEEDER_DEFAULT_RPM;
    feeder->setTargetRPM(speed);
}

void BurstFeederCommand::end(bool) {
    feeder->setTargetRPM(0);
}

bool BurstFeederCommand::isReady() {
    return true;
}

bool BurstFeederCommand::isFinished() const {
    int elapsedTotal = feeder->getTotalLimitCount() - initialTotalBallCount;

    return elapsedTotal >= burstLength;
}

}  // namespace src::Feeder