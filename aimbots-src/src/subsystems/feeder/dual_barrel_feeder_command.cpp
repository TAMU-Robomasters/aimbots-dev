#include "dual_barrel_feeder_command.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

DualBarrelFeederCommand::DualBarrelFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    std::array<BarrelID, 2> BARREL_IDS,
    float speed,
    float unjamSpeed,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      BARREL_IDS(BARREL_IDS),
      speed(speed),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      unjamSpeed(-unjamSpeed)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void DualBarrelFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

void DualBarrelFeederCommand::execute() {
        if (fabs(feeder->getRPM(0)) <= 10.0f && startupThreshold.execute()) {
            feeder->setTargetRPM(unjamSpeed);
            unjamTimer.restart(UNJAM_TIMER_MS);
        }

        if (unjamTimer.execute()) {
            feeder->setTargetRPM(speed);
            startupThreshold.restart(500);
        }
}

void DualBarrelFeederCommand::end(bool) { feeder->setTargetRPM(0.0f); }

bool DualBarrelFeederCommand::isReady() {
    return (true);
}

bool DualBarrelFeederCommand::isFinished() const {
    return (false);
}

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE