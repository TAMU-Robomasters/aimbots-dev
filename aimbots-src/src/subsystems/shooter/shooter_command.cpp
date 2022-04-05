#include "subsystems/shooter/shooter_command.hpp"

#include "drivers.hpp"
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

//#ifndef TARGET_ENGINEER

namespace src::Shooter {

ShooterCommand::ShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter) {
    this->drivers = drivers;
    this->shooter = shooter;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void ShooterCommand::initialize() {
    // declare fixed 8500 RPM target until command is descheduled
    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, 0.0f);
    // 3000 is a reasonable speed
}

// set the flywheel to a certain speed once the command is called
void ShooterCommand::execute() {
    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, 100000.0f);
}

void ShooterCommand::end(bool) {
    // declare motors stop w/ PID upon command deschedule
    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, 0.0f);
}

bool ShooterCommand::isReady() {
    return true;
}

bool ShooterCommand::isFinished() const {
    return false;
}
}  // namespace src::Shooter