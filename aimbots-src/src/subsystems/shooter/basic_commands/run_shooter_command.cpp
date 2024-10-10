#include "subsystems/shooter/basic_commands/run_shooter_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "subsystems/shooter/control/shooter.hpp"
#include "drivers.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter{
    RunShooterCommand::RunShooterCommand(
        src::Drivers* drivers,
        ShooterSubsystem* shooter)
        : drivers(drivers),
        shooter(shooter)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
    }

    void RunShooterCommand::initialize() {
        shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, static_cast<float>(4000));
    }

    void RunShooterCommand::execute() {
        shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
    }

    void RunShooterCommand::end(bool interrupted) {
        
    }

    bool RunShooterCommand::isReady() {
        return true;
    }

    bool RunShooterCommand::isFinished() const {
        return false;
    }
}

#endif