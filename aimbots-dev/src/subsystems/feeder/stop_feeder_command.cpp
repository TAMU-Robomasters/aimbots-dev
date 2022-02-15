#include "stop_feeder_command.hpp"

namespace src::Feeder {
    StopFeederCommand::StopFeederCommand(src::Drivers * drivers, FeederSubsystem * feeder)
        : drivers(drivers),feeder(feeder) {
            addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
        }
    
    void StopFeederCommand::initialize() {}

    void StopFeederCommand::execute() {
          feeder->setDesiredOutput(0); 
    }

    void StopFeederCommand::end(bool) {}

    bool StopFeederCommand::isReady() {
        return true;
    }

    bool StopFeederCommand::isFinished() const {
        return false;
    }
}