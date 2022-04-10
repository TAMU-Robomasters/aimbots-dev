#include "subsystems/shooter/brake_shooter_command.hpp"

#include "drivers.hpp"
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

//#ifndef TARGET_ENGINEER

namespace src::Shooter {

BrakeShooterCommand::BrakeShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter) {
    this->drivers = drivers;
    this->shooter = shooter;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void BrakeShooterCommand::initialize() {
}

// set the flywheel to a certain speed once the command is called
void BrakeShooterCommand::execute() {
    shooter->ForAllShooterMotors(&ShooterSubsystem::setDesiredOutput, -100.0f);
}

void BrakeShooterCommand::end(bool) {
}

bool BrakeShooterCommand::isReady() {
    return true;
}

bool BrakeShooterCommand::isFinished() const {
    float speed = shooter->getMotorSpeed(TOP);
    return (speed>0 ? speed : -speed < 100.0f); //couldn't find math.abs()
}

}  // namespace src::Shooter

//#endif //#ifndef TARGET_ENGINEER