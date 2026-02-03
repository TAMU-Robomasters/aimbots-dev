#include "encoder_single_shot_command.hpp"

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE
#ifdef NO_LIMIT_COMPATIBLE

namespace src::Feeder {

EncoderSingleShotCommand::EncoderSingleShotCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    int UNJAM_TIMER_MS,
    int SINGLE_SHOT_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      SINGLE_SHOT_MS(SINGLE_SHOT_MS) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void EncoderSingleShotCommand::initialize() {
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
    singleShotTimer.restart(SINGLE_SHOT_MS);
    limitswitchInactive.restart(0);
}

void EncoderSingleShotCommand::execute() {
    if (!singleShotTimer.isExpired()) {
        feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::activateFeederMotor);

        if (fabs(feeder->getCurrentRPM(0)) <= 5.0f && unjamTimer.isExpired()) {
            unjamTimer.restart(UNJAM_TIMER_MS);
        }

        if (!unjamTimer.isExpired() && startupThreshold.isExpired()) {
            feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::unjamFeederMotor);
        } else {
            feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::activateFeederMotor);
        }
    } else {
        feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    }
}

void EncoderSingleShotCommand::end(bool) { feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor); }

bool EncoderSingleShotCommand::isReady() { return true; }

bool EncoderSingleShotCommand::isFinished() const { return false; }



}
#endif
#endif  // #ifdef FEEDER_COMPATIBLE
