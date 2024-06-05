#pragma once

#include "feeder_limit_command.hpp"

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/feeder.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

bool limitpressed = false;
bool isCurrSemiauto = false;
bool isPrevSemiauto = false;
bool isFiring = false;
int limitSwitchDownTime;

FeederLimitCommand::FeederLimitCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      isPressed(false),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FeederLimitCommand::initialize() {
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    limitswitchInactive.restart(0);
}

void FeederLimitCommand::execute() {
    // Updates the limit switch state (is pressed or not)
    limitpressed = feeder->getPressed();
    // Updates the previous controller switch state (is up or not)
    isPrevSemiauto = isCurrSemiauto;
    // Updates the current controller switch state
    isCurrSemiauto = drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;
    // States how long the limit switch is ignored when firing a projectile
    limitSwitchDownTime = 350;
    // States the speed of the feeder wheel when firing
    // Checks if the limit switch is pressed & is "not killed" (refer to )
    if (limitswitchInactive.isExpired()) {
        isFiring = false;
    }
    // if the fire mode has been activated (with timer semiauto) then check if your still in the other timers range to keep
    // shooting, else set the feeder to 0 until reset timers
    if (!isFiring) {
        feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::activateFeederMotor);
        if (!limitpressed) {
            feeder->ForFeederMotorGroup(SECONDARY, &FeederSubsystem::activateFeederMotor);
        } else {
            feeder->ForFeederMotorGroup(SECONDARY, &FeederSubsystem::deactivateFeederMotor);
        }
    }

    // Checks if the controller is in its semiautomatic state
    //      i.e. controller switch goes from mid to up
    // If so, it (should) launch the currently loaded ball and turn off until the controller exits semi auto (ie cSwitch goes
    // to mid)
    if (isCurrSemiauto and !isPrevSemiauto) {
        feeder->ForFeederMotorGroup(PRIMARY, &FeederSubsystem::activateFeederMotor);
        isFiring = true;
        limitswitchInactive.restart(limitSwitchDownTime);
    }
}
void FeederLimitCommand::end(bool) { feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor); }

bool FeederLimitCommand::isReady() { return true; }

bool FeederLimitCommand::isFinished() const { return false; }

}  // namespace src::Feeder
#endif