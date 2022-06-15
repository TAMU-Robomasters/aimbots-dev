#include "run_feeder_command.hpp"

namespace src::Feeder {

static constexpr float BARREL_HEAT_MAX_PERCENTAGE = 0.90f;

RunFeederCommand::RunFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers),
      feeder(feeder),
      canShoot(true),
      speed(0)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void RunFeederCommand::initialize() {
    speed = 0.0f;
    feeder->setTargetRPM(0.0f);
}

void RunFeederCommand::execute() {
    canShoot = isOverMaxHeatPercentage(drivers, BARREL_HEAT_MAX_PERCENTAGE);

    speed = FEEDER_DEFAULT_RPM;
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