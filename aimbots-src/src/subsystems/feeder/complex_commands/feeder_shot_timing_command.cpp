#include "feeder_shot_timing_command.hpp"

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "communicators/jetson/jetson_protocol.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

bool currSTFireStateDisplay = false;
FeederState feederStateDisplay = FeederState::undefined;
bool limitSwitchFRDisplay = false;


FeederShotTimingCommand::FeederShotTimingCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS),
      currentFeederState(FeederState::initialize)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FeederShotTimingCommand::initialize() {
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
    limitswitchInactive.restart(0);
    currSwitchState = feeder->getPressed();
    prevSwitchState = feeder->getPressed();
}


void FeederShotTimingCommand::execute() {
    //TODO: implement logic for robots with more than one feeder
    // if (fabs(feeder->getCurrentRPM(0)) <= 5.0f && unjamTimer.isExpired()){
    //     unjamTimer.restart(UNJAM_TIMER_MS);
    //     currentFee
    // }

    feederStateDisplay = currentFeederState;
    limitSwitchFRDisplay = feeder->getPressed();
    prevSwitchState = currSwitchState;
    currSwitchState = feeder->getPressed();

    switch (currentFeederState) {
    case FeederState::initialize:
        //! potential bug where limit switch is pressed but there's not enough projectiles in ball path
        if (feeder->getPressed()) { // already primed
            currentFeederState = FeederState::primed;
        }
        currentFeederState = FeederState::priming; // start feeding projectiles
        break;
    
    case FeederState::priming: // spin feeder until we hit the limit switch
        if (feeder->getPressed()) { // projectiles reached limit switch
            feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::deactivateFeederMotor);
            currentFeederState = FeederState::primed;
            break;
        }
        feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::activateFeederMotor);
        break;
    case FeederState::primed:
        currFireState = (
            drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP 
            || drivers->remote.getMouseL()==true
            || drivers->cvCommunicator.shouldFire()
        );  
        currSTFireStateDisplay = currFireState;      
        if (currFireState && !prevFireState) { // boolean expression detects rising edge
            currentFeederState = FeederState::firing;
        }
        prevFireState = currFireState;
        break;
    case FeederState::firing: // move feeder until current projectile is shot
        if (currSwitchState && !prevSwitchState) { // current projectile left. New projectile primed
            feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::deactivateFeederMotor);
            currentFeederState = FeederState::primed;
            break;
        }
        feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::activateFeederMotor);
        break;
    default:
        break;
    }
}
void FeederShotTimingCommand::end(bool) { feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor); }

bool FeederShotTimingCommand::isReady() { return true; }

bool FeederShotTimingCommand::isFinished() const { return false; }

}  // namespace src::Feeder
#endif