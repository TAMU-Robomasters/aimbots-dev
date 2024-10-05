#include "stop_feeder_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

    StopFeederCommand::StopFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers), feeder(feeder)
    {   
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    }
    
    void StopFeederCommand::initialize() {
        feeder->ForAllFeederMotors(&FeederSubsystem::setTargetRPM, static_cast<float>(0));
    }
    
    void StopFeederCommand::execute() {
        feeder->ForAllFeederMotors(&FeederSubsystem::updateMotorVelocityPID);
    }

    void StopFeederCommand::end(bool interrupted) {
        if (interrupted) {
            feeder->ForAllFeederMotors(&FeederSubsystem::setTargetRPM, static_cast<float>(0));
        }
    }
    
    bool StopFeederCommand::isReady() {
        return true;
    }

    bool StopFeederCommand::isFinished() const {
        return false;
    }
}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE
