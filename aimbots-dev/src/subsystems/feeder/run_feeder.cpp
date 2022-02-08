#include "run_feeder.hpp"

namespace src::Feeder {
    RunFeeder::RunFeeder(src::Drivers * drivers, FeederSubsystem * feeder, float speed)
        : drivers(drivers),feeder(feeder) {
            addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
        }
    
    void RunFeeder::initialize() {}

    void RunFeeder::execute() {
          feeder->setDesiredOutput(speed); 
    }

    void RunFeeder::end(bool) {}

    bool RunFeeder::isReady() {
        return true;
    }

    bool RunFeeder::isFinished() const {
        return false;   //finished condition (button released) or their api is nice and we don't have to
    }
}