#include "full_auto_feeder_command.hpp"

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float speed, float acceptableHeatThreshold)
    : drivers(drivers),
      feeder(feeder),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      unjamSpeed(-speed / 2.0f) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    unjamTimer.stop();
}

void FullAutoFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(1000);
    unjamTimer.restart(0);
}

void FullAutoFeederCommand::execute() {
    if(startupThreshold.execute()){
        startupThreshold.stop();
    }
    if (unjamTimer.execute()) {
        unjamTimer.stop();
    }

    if (startupThreshold.isStopped()) {
        if ((fabs(feeder->getCurrentRPM()) < fabs(speed / 2.0f)) && (unjamTimer.isStopped())) {
            unjamTimer.restart(300);
        }
    }

    if (!unjamTimer.isStopped()) {
        feeder->setTargetRPM(-speed);
    } else {
        feeder->setTargetRPM(speed);
    }
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