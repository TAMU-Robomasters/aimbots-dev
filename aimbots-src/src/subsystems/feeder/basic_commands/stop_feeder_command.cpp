#include "stop_feeder_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

    StopFeederCommand::StopFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers), feeder(feeder)
    {   
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    }
    
    void FullAutoFeederCommand::initialize() {
        feeder->ForAllShooterMotors(&FeederSubsystem::setTargetRPM, static_cast<float>(0));
    }
    
    void FullAutoFeederCommand::execute() {
        feeder->ForAllShooterMotors(&FeederSubsystem::updateMotorVelocityPID);
    }

    void FullAutoFeederCommand::end(bool interrupted) {
        if (interrupted) {
            feeder->ForAllShooterMotors(&FeederSubsystem::setTargetRPM, static_cast<float>(0));
        }
    }
    
    bool FullAutoFeederCommand::isReady() {
        return false;
    }

    bool FullAutoFeederCommand::isFinished() {
        return true;
    }
}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE
