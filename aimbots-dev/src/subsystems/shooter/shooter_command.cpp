#include "drivers.hpp"
#include "subsystems/shooter/shooter_command.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Shooter {

    ShooterCommand::ShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter){
        this->drivers = drivers;
        this->shooter = shooter;
    }
    void ShooterCommand::initialize(){}

    //initialize RPM
    float RPM; 
    // set the flywheel to a certain speed once the command is called
    void ShooterCommand::execute(){
        shooter->calculateShooter(RPM);
    }
    void ShooterCommand::end(bool interrupted){}

    bool ShooterCommand::isReady(){
        return true;
    }

    bool ShooterCommand::isFinished() const {
        return false;
    }

}//namespace src::Shooter