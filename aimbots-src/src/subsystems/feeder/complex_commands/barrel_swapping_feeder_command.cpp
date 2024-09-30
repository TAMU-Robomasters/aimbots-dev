#include "barrel_swapping_feeder_command.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

BarrelSwappingFeederCommand::BarrelSwappingFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    bool& barrelMovingFlag,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      barrelMovingFlag(barrelMovingFlag),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS)//
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void BarrelSwappingFeederCommand::initialize() {
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

float swappableHeatLimitDisplay = 0;
float swappableCurrentHeatDisplay = 0;

void BarrelSwappingFeederCommand::execute() {
    if (refHelper->canCurrBarrelShootSafely() && !barrelMovingFlag) {
        if (unjamTimer.execute()) {
            feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::activateFeederMotor);
            startupThreshold.restart(500);
        }

        if (fabs(feeder->getCurrentRPM(0)) <= 10.0f && startupThreshold.execute()) {
            feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::unjamFeederMotor);
            unjamTimer.restart(UNJAM_TIMER_MS);
        }

    } else {
        feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
        unjamTimer.restart(0);
    }

    swappableHeatLimitDisplay = refHelper->getCurrBarrelLimit();
    swappableCurrentHeatDisplay = refHelper->getCurrBarrelHeat();
}

void BarrelSwappingFeederCommand::end(bool) { feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor); }

bool BarrelSwappingFeederCommand::isReady() { return (refHelper->canCurrBarrelShootSafely() && !barrelMovingFlag); }

bool BarrelSwappingFeederCommand::isFinished() const {
    // return (!refHelper->isCurrBarrelHeatUnderLimit(acceptableHeatThreshold) || barrelMovingFlag);
    return false;
}

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE