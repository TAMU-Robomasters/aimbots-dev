#include "full_auto_feeder_command.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    float speed,
    float unjamSpeed,
    uint8_t projectileBuffer,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      projectileBuffer(projectileBuffer),
      speed(speed),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      unjamSpeed(-unjamSpeed)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

bool isCommandRunningDisplay = false;
uint8_t projectileAllowanceDisplay = 18;

int64_t maxRotationDisplay = 15;
int64_t currentRotationDisplay = 12;

/*
On initialization, check the amount of heat that is left, divide that by
the amount of heat gained by projectiles, (10/projectile for 17mm according to
ref_helper_turreted.hpp). Use the number of projectiles to determine how much
the feeder motor can rotate from this command until stopping
to avoid shooting extra projectiles
*/

void FullAutoFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);

    // When the command is scheduled, calculating the remaining projectiles
    // that can be shot based on heat
    uint8_t projectilesRemaining = refHelper->getRemainingProjectiles() - projectileBuffer;

    projectileAllowanceDisplay = projectilesRemaining;
    // get the maximum rotations the feeder can make based on how many projectiles
    // it shoots in one rotation
    float maxRotations = projectilesRemaining / PROJECTILES_PER_FEEDER_ROTATION;

    // Get the maximum absolute position the motor can get to
    int64_t encoderChangeThreshold = DJIMotor::ENC_RESOLUTION * maxRotations;
    antiOverheatEncoderThreshold = feeder->getEncoderUnwrapped() + encoderChangeThreshold;
}

uint16_t lastHeatDisplay = 0;
uint16_t heatLimitDisplay = 0;
float lastProjectileSpeedDisplay = 0.0f;

void FullAutoFeederCommand::execute() {
    isCommandRunningDisplay = true;

    // If the absolute encoder position is past the threshold to not
    // overheat, set the RPM to 0, otherwise run as normal
    maxRotationDisplay = antiOverheatEncoderThreshold;
    currentRotationDisplay = feeder->getEncoderUnwrapped();

    if (feeder->getEncoderUnwrapped() >= antiOverheatEncoderThreshold) {
        feeder->setTargetRPM(0.0f);
    } else {
        if (fabs(feeder->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
            feeder->setTargetRPM(unjamSpeed);
            unjamTimer.restart(UNJAM_TIMER_MS);
        }

        if (unjamTimer.execute()) {
            feeder->setTargetRPM(speed);
            startupThreshold.restart(500);
        }

        lastHeatDisplay = refHelper->getCurrBarrelHeat();
        heatLimitDisplay = refHelper->getCurrBarrelLimit();
        lastProjectileSpeedDisplay = refHelper->getLastProjectileSpeed();
    }
}

void FullAutoFeederCommand::end(bool) {
    feeder->setTargetRPM(0.0f);
    isCommandRunningDisplay = false;
}

bool FullAutoFeederCommand::isReady() { return true; }

bool FullAutoFeederCommand::isFinished() const { return false; }

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE