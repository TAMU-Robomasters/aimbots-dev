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
    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, 0.0f);
}

// set the flywheel to a certain speed once the command is called
void ShooterDefaultCommand::execute() {
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