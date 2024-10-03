#include "subsystems/shooter/basic_commands/stop_shooter_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "subsystems/shooter/control/shooter.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {
    StopShooterCommand::StopShooterCommand(
        src::Drivers *drivers,
        ShooterSubsystem *shooter
    ) : drivers(drivers),
        shooter(shooter)
    
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
    }

void StopShooterCommand::initialize(){
    shooter->initialize();
    shooter->setTargetRPM(RIGHT, 0);
    shooter->setTargetRPM(RIGHT, 0);
}

void StopShooterCommand::execute(){
    shooter->refresh();
}

void StopShooterCommand::end(bool interrupted){
}

bool StopShooterCommand::isReady() {
    return true;
}

bool StopShooterCommand::isFinished() const{
    return false;
}

}; //namespace src::Shooter


#endif //#ifdef SHOOTER_COMPATIBLE