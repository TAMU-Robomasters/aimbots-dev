#include "subsystems/grabber/suction_command.hpp"

#ifdef GRABBER_COMPATIBLE

namespace src::Grabber {

Suction_Command::Suction_Command(tap::Drivers* drivers, GrabberSubsystem* grabber) {
    this->drivers = drivers;
    this->grabber = grabber;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(grabber));
}

bool isCommandRunningDisplay = false;
void Suction_Command::initialize() { grabber->deactivate(); }

/**
 * action here
 */
void Suction_Command::execute() {
    // if (duty_cycle > 1.0f) {
    //     duty_cycle = 1.0f;
    // } else if (duty_cycle < 0.0f) {
    //     duty_cycle = 0.0f;
    // }
    bool isCommandRunningDisplay = true;
    grabber->activate();
    // grabber->unknown();
}

void Suction_Command::end(bool interrupted) {
    isCommandRunningDisplay = false;
    grabber->deactivate();
    // drivers->pwm.write(0.0f, GRABBER_PIN);
}

bool Suction_Command::isReady() { return true; }

bool Suction_Command::isFinished() const { return false; }

}  // namespace src::Grabber

#endif