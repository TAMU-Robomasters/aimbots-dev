#include "subsystems/shooter/basic_commands/run_shooter_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "subsystems/shooter/control/shooter.hpp"
#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {
    RunShooterCommand::RunShooterCommand(
        src::Drivers *drivers,
        ShooterSubsystem *shooter
    ) : drivers(drivers),
        shooter(shooter)
    
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
    }

void RunShooterCommand::initialize() {
    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, 5000.0f);
    // shooter->setTargetRPM(LEFT, 5000);
    // shooter->setTargetRPM(RIGHT, 5000);
}

void RunShooterCommand::execute() {
    shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
    // shooter->updateMotorVelocityPID(LEFT);
    // shooter->updateMotorVelocityPID(RIGHT);
}

void RunShooterCommand::end(bool interrupted){
    //stop_shooter_command();
}

bool RunShooterCommand::isReady() {
    return true;
}

bool RunShooterCommand::isFinished() const{
    return false;
}

}; //namespace src::Shooter


#endif //#ifdef SHOOTER_COMPATIBLE