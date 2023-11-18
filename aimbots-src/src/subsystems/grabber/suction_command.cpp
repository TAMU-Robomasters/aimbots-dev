#include "subsystems/grabber/suction_command.hpp"

#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"



#ifdef GRABBER_COMPATIBLE 

namespace src:: Grabber {

GrabberSubsystem::GrabberSubsystem(src::Drivers* dirvers, tap::gpio::Pwm* pwm, GrabberSubsystem* grabber, float duty_cycle){
    this->drivers = drivers;
    this->grabber = grabber;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(grabber));
}

void GrabberSubsystem::initialize() {
    grabber->activate(0.0f);

}

void GrabberSubsystem::execute() {
    grabber->activate(duty_cycle);

}

void GrabberSubsystem::end() {

}

bool GrabberSubsystem::isReady() { return true; }

bool GrabberSubsystem::isFinished() const { return false; }

} 

#endif 
