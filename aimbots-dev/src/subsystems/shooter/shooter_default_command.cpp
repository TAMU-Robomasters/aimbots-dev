#include "subsystems/shooter/shooter_default_command.hpp"

#include "drivers.hpp"
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

//#ifndef TARGET_ENGINEER

namespace src::Shooter {

ShooterDefaultCommand::ShooterDefaultCommand(src::Drivers* drivers, ShooterSubsystem* shooter) {
    this->drivers = drivers;
    this->shooter = shooter;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void ShooterDefaultCommand::initialize() {
}

// set the flywheel to a certain speed once the command is called
void ShooterDefaultCommand::execute() {
    drivers->leds.set(tap::gpio::Leds::B, false);
    drivers->leds.set(tap::gpio::Leds::C, true);
    drivers->leds.set(tap::gpio::Leds::D, true);
    drivers->leds.set(tap::gpio::Leds::E, false);
    drivers->leds.set(tap::gpio::Leds::F, true);
    drivers->leds.set(tap::gpio::Leds::G, true);
    drivers->leds.set(tap::gpio::Leds::H, false);

    // stop >:(
    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, 0.0f);
    // 3000 is a reasonable speed
}

void ShooterDefaultCommand::end(bool) {
}

bool ShooterDefaultCommand::isReady() {
    return true;
}

bool ShooterDefaultCommand::isFinished() const {
    return false;
}

}  // namespace src::Shooter

//#endif //#ifndef TARGET_ENGINEER