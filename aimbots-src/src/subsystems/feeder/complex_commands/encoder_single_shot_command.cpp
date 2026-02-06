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
    int64_t SINGLE_SHOT_ENCODER_TICKS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      SINGLE_SHOT_ENCODER_TICKS(SINGLE_SHOT_ENCODER_TICKS),
      singleShotStartEncoder(0),
      singleShotDirectionSign(1) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void EncoderSingleShotCommand::initialize() {
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
    singleShotStartEncoder = feeder->getEncoderUnwrapped(0);
    singleShotDirectionSign = sgn(FEEDER_NORMAL_RPMS[0]);
    if (singleShotDirectionSign == 0) {
        singleShotDirectionSign = 1;
    }
    limitswitchInactive.restart(0);
}

int64_t encoderDeltaWatch = 0;
float const_change;

void EncoderSingleShotCommand::execute() {
    const int64_t encoderDelta = (feeder->getEncoderUnwrapped(0) - singleShotStartEncoder) * singleShotDirectionSign;

    encoderDeltaWatch = encoderDelta;
    const_change = 819.0f;

    if (encoderDelta < SINGLE_SHOT_ENCODER_TICKS) {
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
