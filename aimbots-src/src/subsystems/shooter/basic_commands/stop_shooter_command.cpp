#include "subsystems/shooter/basic_commands/stop_shooter_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "subsystems/shooter/control/shooter.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {
    StopShooterCommand::StopShooterCommand(
        src::Drivers* drivers,
        ShooterSubsystem* shooter
    )
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

    void StopShooterCommand::end(bool) { 
        shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, static_cast<float>(0));
    }
    
    bool StopShooterCommand::isReady() { 
        return true; 
    }

    bool StopShooterCommand::isFinished() const {
        return false;
    }
}

#endif
    

