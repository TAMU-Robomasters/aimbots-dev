#include "subsystems/shooter/basic_commands/stop_shooter_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "subsystems/shooter/control/shooter.hpp"



namespace src::Shooter {
    StopShooterCommand::StopShooterCommand(
        src::Drivers* drivers,
        ShooterSubsystem* shooter
    )
    : drivers(drivers),
      shooter(shooter)
    
{

}


    
void StopsShooterCommand::initialize() {}

    void StopShooterCommand::execute() {}
    void StopShooterCommand::end(bool) {}
    bool StopShooterCommand::isReady() { return true; }

    bool StopShooterCommand::isFinished() const {}

    const char* StopShooterCommand::getName() const override { return "stops shooter"; }



}
    

