#include "run_feeder_command_commmand.hpp"

namespace src::Feeder {
    RunFeederCommand::RunFeederCommand(src::Drivers * drivers, FeederSubsystem * feeder, float speed)
        : drivers(drivers),feeder(feeder) {
            addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
        }
    
    void RunFeederCommand::initialize() {}

    void RunFeederCommand::execute() {
          feeder->setDesiredOutput(speed); 
    }

    void RunFeederCommand::end(bool) {}

    bool RunFeederCommand::isReady() {
        return true;
    }

    bool RunFeederCommand::isFinished() const {
        return false;                       //finished condition (button released) or their api is nice and we don't have to
    }
}