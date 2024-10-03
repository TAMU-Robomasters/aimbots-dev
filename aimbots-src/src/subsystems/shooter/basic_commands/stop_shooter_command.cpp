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

void initialize() {
    Shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, static_cast<float>(0));
}

void execute() {
    Shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
}

void end(bool interrupted) {
    Shooter->ForAllShooterMotors(&ShooterSubsystem::setDesiredOutputToMotor, static_cast<float>(0))
}

bool isFinished() const {
    return Shooter->getHighestMotorSpeed() < 60;
}

bool isReady() { 
    return true;
}
}

#endif