#include "full_auto_feeder_command.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    uint8_t projectileBuffer,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      projectileBuffer(projectileBuffer),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

bool isCommandRunningDisplay = false;

int64_t overheatThresholdDisplay = 15;
int64_t currentRotationDisplay = 12;

/*
On initialization, check the amount of heat that is left, divide that by
the amount of heat gained by projectiles, (10/projectile for 17mm according to
ref_helper_turreted.hpp). Use the number of projectiles to determine how much
the feeder motor can rotate from this command until stopping
to avoid shooting extra projectiles
*/

/*
When initializing, check how much "heat" we have left,

*/

void FullAutoFeederCommand::initialize() {
    feeder->setTargetRPM(0, 0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);

    // When the command is scheduled, calculating the remaining projectiles
    // that can be shot based on heat
    uint64_t encoderChangeThreshold = refHelper->getAllowableFeederRotation(projectileBuffer);
    overheatThresholdDisplay = encoderChangeThreshold;
    antiOverheatEncoderThreshold = feeder->getEncoderUnwrapped() + encoderChangeThreshold;
}

// uint16_t lastHeatDisplay = 0;
// uint16_t heatLimitDisplay = 0;
float lastProjectileSpeedDisplay = 0.0f;

void FullAutoFeederCommand::execute() {
    isCommandRunningDisplay = true;

    // If the absolute encoder position is past the threshold to not
    // overheat, set the RPM to 0, otherwise run as normal
    currentRotationDisplay = feeder->getEncoderUnwrapped(0);

    if (/*false && */ feeder->getEncoderUnwrapped() >= antiOverheatEncoderThreshold) {
        feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    } else {
        if (fabs(feeder->getCurrentRPM(0)) <= 10.0f && startupThreshold.execute()) {
            feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::unjamFeederMotor);
            unjamTimer.restart(UNJAM_TIMER_MS);
        }

        if (unjamTimer.execute()) {
            feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::activateFeederMotor);
            startupThreshold.restart(500);
        }

        // lastHeatDisplay = refHelper->getCurrBarrelHeat();
        // heatLimitDisplay = refHelper->getCurrBarrelLimit();
        lastProjectileSpeedDisplay = refHelper->getLastProjectileSpeed();
    }
}

void FullAutoFeederCommand::end(bool) {
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    isCommandRunningDisplay = false;
}

bool FullAutoFeederCommand::isReady() { return true; }

bool FullAutoFeederCommand::isFinished() const { return false; }

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE