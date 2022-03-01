#include "run_feeder_command.hpp"

namespace src::Feeder {
    RunFeederCommand::RunFeederCommand(src::Drivers * drivers, FeederSubsystem * feeder, float speed)
        : drivers(drivers),feeder(feeder),speed(speed) {
            addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
        }
    
    void RunFeederCommand::initialize() {
        feeder->updateRPM(speed);
    }

    void RunFeederCommand::execute() {
        drivers->leds.set(tap::gpio::Leds::A,true);
        drivers->leds.set(tap::gpio::Leds::B,false);
        drivers->leds.set(tap::gpio::Leds::C,true);
        drivers->leds.set(tap::gpio::Leds::D,false);
        drivers->leds.set(tap::gpio::Leds::E,true); 
        drivers->leds.set(tap::gpio::Leds::F,false);
        drivers->leds.set(tap::gpio::Leds::G,true);
        drivers->leds.set(tap::gpio::Leds::H,false); 
        feeder->setDesiredOutput();
    }

    void RunFeederCommand::end(bool) {}

    bool RunFeederCommand::isReady() {
        return true;
    }

    bool RunFeederCommand::isFinished() const {
        return false;                       //finished condition (button released) or their api is nice and we don't have to
    }
}