#include "encoder_single_shot_command.hpp"

#ifdef FEEDER_COMPATIBLE
#ifdef NO_LIMIT

namespace src::Feeder {

EncoderSingleShotCommand::EncoderSingleShotCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void EncoderSingleShotCommand::initialize() {
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
    limitswitchInactive.restart(0);
}

void EncoderSingleShotCommand::execute() {
    
}

void EncoderSingleShotCommand::end(bool) { feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor); }

bool EncoderSingleShotCommand::isReady() { return true; }

bool EncoderSingleShotCommand::isFinished() const { return false; }


}
#endif
#endif  // #ifdef FEEDER_COMPATIBLE