#include "subsystems/shooter/brake_shooter_command.hpp"
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/tools/common_types.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {
static int highestSpeedDebug = 0;
BrakeShooterCommand::BrakeShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter, float brakePower)
    : brakePID(brakePower, 0, 0, 0.0f, 2000.0f) {
    this->drivers = drivers;
    this->shooter = shooter;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void BrakeShooterCommand::initialize() {}

// set the flywheel to a certain speed once the command is called
void BrakeShooterCommand::execute() {
    brakePID.update(-shooter->getHighestMotorSpeed());
    float output = brakePID.getValue();
    float highestSpeed = shooter->getHighestMotorSpeed();
    highestSpeedDebug = (int)highestSpeed;
    shooter->ForAllShooterMotors(&ShooterSubsystem::setDesiredOutput, output);
}

void BrakeShooterCommand::end(bool) {}

bool BrakeShooterCommand::isReady() { return true; }

bool BrakeShooterCommand::isFinished() const {
    float speedTolerance = 1000.0f;
    return shooter->getHighestMotorSpeed() < speedTolerance;  // replace with getHighestMotorSpeed()
}

}  // namespace src::Shooter

#endif //#ifdef SHOOTER_COMPATIBLE