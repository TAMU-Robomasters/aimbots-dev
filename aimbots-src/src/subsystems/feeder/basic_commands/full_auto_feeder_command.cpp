#include "full_auto_feeder_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

    FullAutoFeederCommand::FullAutoFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers), feeder(feeder)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    }
    
    void FullAutoFeederCommand::initialize() {
        feeder->ForAllFeederMotors(&FeederSubsystem::setTargetRPM, static_cast<float>(15));
    }
    
    void FullAutoFeederCommand::execute() {
        feeder->ForAllFeederMotors(&FeederSubsystem::updateMotorVelocityPID);
    }

    void FullAutoFeederCommand::end(bool interrupted) {
        if (interrupted) {
            feeder->ForAllFeederMotors(&FeederSubsystem::setTargetRPM, static_cast<float>(0));
        }
    }
    
    bool FullAutoFeederCommand::isReady() {
        return true;
    }

    bool FullAutoFeederCommand::isFinished() const {
        return false;
    }
}

#endif  // #ifdef FEEDER_COMPATIBLE
