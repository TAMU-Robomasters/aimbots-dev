#include "stop_feeder_command.hpp"

namespace src::Feeder {
StopFeederCommand::StopFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers), feeder(feeder) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void StopFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
}

void StopFeederCommand::execute() {
    // drivers->leds.set(tap::gpio::Leds::A, false);
    // drivers->leds.set(tap::gpio::Leds::B, false);
    // drivers->leds.set(tap::gpio::Leds::C, false);
    // drivers->leds.set(tap::gpio::Leds::D, false);
    // drivers->leds.set(tap::gpio::Leds::E, false);
    // drivers->leds.set(tap::gpio::Leds::F, true);
    // drivers->leds.set(tap::gpio::Leds::G, false);
    // drivers->leds.set(tap::gpio::Leds::H, true);
    feeder->setTargetRPM(0.0f);
}

void StopFeederCommand::end(bool) {}

bool StopFeederCommand::isReady() {
    return true;
}

bool StopFeederCommand::isFinished() const {
    return false;
}
}  // namespace src::Feeder