#include "subsystems/grabber/stop_suction_command.hpp"

#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#include "grabber.hpp"

#ifdef GRABBER_COMPATIBLE 

namespace src:: Grabber {

GrabberSubsystem::GrabberSubsystem(src::Drivers* dirvers, tap::gpio::Pwm* pwm){

}
void GrabberSubsystem::initialize() {
}

void GrabberSubsystem::execute() {

}

void GrabberSubsystem::end() {

}

bool GrabberSubsystem::isReady() { return true; }

bool GrabberSubsystem::isFinished() const {  }

};  

#endif 
