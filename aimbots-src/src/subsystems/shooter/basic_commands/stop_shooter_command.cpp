#include "utils/tools/robot_specific_inc.hpp"
#ifdef SHOOTER_COMPATIBLE
#include "subsystems/shooter/basic_commands/stop_shooter_command.hpp"
#include "subsystems/shooter/basic_commands/run_shooter_command.hpp"

namespace src::Shooter {

StopShooterCommand::StopShooterCommand(
    src::Drivers* drivers,
    ShooterSubsystem* shooter)
    : drivers(drivers),
      shooter(shooter)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void StopShooterCommand::initialize() {
    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, static_cast<float>(0));
}

void StopShooterCommand::execute() {
    shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
}

void StopShooterCommand::end(bool interrupted) {
    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, static_cast<float>(0));
}

bool StopShooterCommand::isFinished() const {
    return shooter->getHighestMotorSpeed() < 1;
}

bool StopShooterCommand::isReady() { 
    return true;
}
}

#endif