#include "utils/tools/robot_specific_inc.hpp"
#ifdef SHOOTER_COMPATIBLE
#include "subsystems/shooter/basic_commands/run_shooter_command.hpp"

namespace src::Shooter {

RunShooterCommand::RunShooterCommand(
    src::Drivers* drivers,
    ShooterSubsystem* shooter)
    : drivers(drivers),
      shooter(shooter)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void RunShooterCommand::initialize() {
    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, static_cast<float>(4300));
}

void RunShooterCommand::execute() {
    shooter->ForAllShooterMotor(&ShooterSubsystem::updateMotorVelocityPID)
}

void RunShooterCommand::end(bool interrupted) { }

bool RunShooterCommand::isFinished() const {
    return false;
}

bool RunShooterCommand::isReady() { 
    return true;
}
}

#endif