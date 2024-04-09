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
    uint8_t projectileBuffer,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      BARREL_IDS(BARREL_IDS),
      projectileBuffer(projectileBuffer),
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

    // When the command is scheduled, calculating the remaining projectiles
    // that can be shot based on heat
    uint8_t projectilesRemaining = refHelper->getRemainingProjectiles() - projectileBuffer;

    // get the maximum rotations the feeder can make based on how many projectiles
    // it shoots in one rotation
    float maxRotations = projectilesRemaining / PROJECTILES_PER_FEEDER_ROTATION;

    // Get the maximum absolute position the motor can get to
    int64_t encoderChangeThreshold = DJIMotor::ENC_RESOLUTION * maxRotations;
    antiOverheatEncoderThreshold = feeder->getEncoderUnwrapped() + encoderChangeThreshold;
}

void DualBarrelFeederCommand::execute() {
    // If the absolute encoder position is past the threshold to not
    // overheat, set the RPM to 0, otherwise run as normal
    if (feeder->getEncoderUnwrapped() >= antiOverheatEncoderThreshold) {
        feeder->setTargetRPM(0.0f);
    } else {    
        if (fabs(feeder->getRPM()) <= 10.0f && startupThreshold.execute()) {
            feeder->setTargetRPM(unjamSpeed);
            unjamTimer.restart(UNJAM_TIMER_MS);
        }

        if (unjamTimer.execute()) {
            feeder->setTargetRPM(speed);
            startupThreshold.restart(500);
        }
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