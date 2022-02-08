#include "stop_feeder.hpp"

namespace src::Feeder {
    StopFeeder::StopFeeder(src::Drivers * drivers, FeederSubsystem * feeder)
        : drivers(drivers),feeder(feeder) {
            addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
        }
    
    void StopFeeder::initialize() {}

    void StopFeeder::execute() {
          feeder->setDesiredOutput(0); 
    }

    void StopFeeder::end(bool) {}

    bool StopFeeder::isReady() {
        return true;
    }

    bool StopFeeder::isFinished() const {
        return false;
    }
}