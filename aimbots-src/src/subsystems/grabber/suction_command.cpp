#include "subsystems/grabber/suction_command.hpp"

#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"



#ifdef GRABBER_COMPATIBLE 

namespace src:: Grabber {

Suction_Command::Suction_Command(tap::Drivers* drivers, GrabberSubsystem* grabber){
    this->drivers = drivers;
    this->grabber = grabber;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(grabber));
}

void Suction_Command::initialize() {
    grabber->deactivate();

}

void Suction_Command::execute() {
    // if (drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwicthState::UP) {

    // }
    // if (duty_cycle > 1.0f) {
    //     duty_cycle = 1.0f;
    // } else if (duty_cycle < 0.0f) {
    //     duty_cycle = 0.0f;
    // }
    grabber->activate();
    // drivers->pwm.write(1.0f, GRABBER_PIN);

}

void Suction_Command::end(bool interrupted) {
    grabber->deactivate();
    // drivers->pwm.write(0.0f, GRABBER_PIN);
}

bool Suction_Command::isReady() { return true; }

bool Suction_Command::isFinished() const { return false; }

} 

#endif 
