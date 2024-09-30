#include "dual_barrel_feeder_command.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

DualBarrelFeederCommand::DualBarrelFeederCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    std::array<BarrelID, 2> BARREL_IDS,
    uint8_t projectileBuffer,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      BARREL_IDS(BARREL_IDS),
      projectileBuffer(projectileBuffer),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void DualBarrelFeederCommand::initialize() {
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);

    // Get the maximum absolute position the motor can get to
    int64_t encoderChangeThreshold = refHelper->getAllowableFeederRotation(projectileBuffer);
    antiOverheatEncoderThreshold = feeder->getEncoderUnwrapped() + encoderChangeThreshold;
}

void DualBarrelFeederCommand::execute() {
    // If the absolute encoder position is past the threshold to not
    // overheat, set the RPM to 0, otherwise run as normal
    if (feeder->getEncoderUnwrapped() >= antiOverheatEncoderThreshold) {
        feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    } else {
        if (fabs(feeder->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
            feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::unjamFeederMotor);
            unjamTimer.restart(UNJAM_TIMER_MS);
        }

        if (unjamTimer.execute()) {
            feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::activateFeederMotor);
            startupThreshold.restart(500);
        }
    }
}

void DualBarrelFeederCommand::end(bool) { feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor); }

bool DualBarrelFeederCommand::isReady() { return (true); }

bool DualBarrelFeederCommand::isFinished() const { return (false); }

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE