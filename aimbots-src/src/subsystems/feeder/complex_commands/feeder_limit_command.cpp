#include "feeder_limit_command.hpp"

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

bool limitPressed = false;
bool currFireState = false;
bool currLoadState = false;
bool watchFire = false;
bool prevFireState = false;
bool prevLoadState = false;
bool isFiring = false;
bool loaderDormant = false;
int limitSwitchDownTime;

FeederLimitCommand::FeederLimitCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FeederLimitCommand::initialize() {
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
    limitswitchInactive.restart(0);
}

void FeederLimitCommand::execute() {
    // Updates the limit switch state (is pressed or not)
    limitPressed = !feeder->getPressed();  // Logic inverted because of a wire oopsie
    // Updates the previous controller switch state (is up or not)
    prevFireState = currFireState;
    // Updates the current controller switch state
    currFireState = (drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP || drivers->remote.getMouseL()==true);
    // States how long the limit switch is ignored when firing a projectile
    limitSwitchDownTime = 300;
    // States the speed of the feeder wheel when firing
    // Checks if the limit switch is pressed & is "not killed"

    prevLoadState = currLoadState;

    if (!limitswitchInactive.isExpired()) {
        limitPressed = false;
        // feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::deactivateFeederMotor);
    }

    // if the fire mode has been activated (with timer semiauto) then check if your still in the other timers range to keep
    // shooting, else set the feeder to 0 until reset timers
    // if (!isFiring) {
    // }

    if (!limitPressed && !loaderDormant) {
        feeder->ForFeederMotorGroup(SECONDARY, &FeederSubsystem::activateFeederMotor);
        if (fabs(feeder->getCurrentRPM(0)) <= 5.0f && unjamTimer.isExpired()){
            unjamTimer.restart(UNJAM_TIMER_MS);
        }
        if (!unjamTimer.isExpired() && startupThreshold.isExpired()) {
            feeder->ForFeederMotorGroup(SECONDARY, &FeederSubsystem::unjamFeederMotor);
            currLoadState=false;
        }else{
            feeder->ForFeederMotorGroup(SECONDARY, &FeederSubsystem::activateFeederMotor);
            currLoadState = true;
        }
        if(prevLoadState == false && currLoadState == true){
            startupThreshold.restart(1000);
        }

        // if (!unjamTimer.execute()) {
        //     feeder->ForFeederMotorGroup(SECONDARY, &FeederSubsystem::activateFeederMotor);
        //     //feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::setFeederCustomMulti, 1.0f);
        //     //feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::activateFeederMotor);
        //     startupThreshold.restart(500);
        // }
    } else {
        feeder->ForFeederMotorGroup(SECONDARY, &FeederSubsystem::deactivateFeederMotor);
        currLoadState = false;
        if (isFiring) {
            feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::deactivateFeederMotor);
        }
        loaderDormant = true;
    }

    // Checks if the controller is in its semiautomatic state
    //      i.e. controller switch goes from mid to up
    // If so, it (should) launch the currently loaded ball and turn off until the controller exits semi auto (ie cSwitch goes
    // to mid)
    if (currFireState && !prevFireState) {
        feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::setFeederCustomMulti, 3.0f);
        isFiring = true;
        loaderDormant = false;
        limitswitchInactive.restart(limitSwitchDownTime);
    }
}
void FeederLimitCommand::end(bool) { feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor); }

bool FeederLimitCommand::isReady() { return true; }

bool FeederLimitCommand::isFinished() const { return false; }

}  // namespace src::Feeder
#endif