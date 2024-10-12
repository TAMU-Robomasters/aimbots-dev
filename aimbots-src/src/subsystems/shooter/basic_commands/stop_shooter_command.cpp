#include "subsystems/shooter/basic_commands/stop_shooter_command.hpp"

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/tools/common_types.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {

StopShooterCommand::StopShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter) {
    this->drivers = drivers;
    this->shooter = shooter;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void StopShooterCommand::initialize() {}

void StopShooterCommand::execute() {
    //shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, 0.0f);
    //shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
}

void StopShooterCommand::end(bool) {}

bool StopShooterCommand::isReady() { return true; }

bool StopShooterCommand::isFinished() const { return false; }

}  // namespace src::Shooter

#endif  //#ifdef SHOOTER_COMPATIBLE