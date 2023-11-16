#include "subsystems/grabber/stop_suction_command.hpp"

#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef GRABBER_COMPATIBLE

namespace src::Grabber {

StopShooterCommand::StopShooterCommand(src::Drivers* drivers, GrabberSubsystem* grabber) {
    this->drivers = drivers;
    this->grabber = grabber;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(grabber));
}

void StopShooterCommand::initialize() {}

// set the flywheel to a certain speed once the command is called
void StopShooterCommand::execute() { grabber->activate(0.0f); }

void StopShooterCommand::end(bool) {}

bool StopShooterCommand::isReady() { return true; }

bool StopShooterCommand::isFinished() const { return false; }

}  // namespace src::Grabber

#endif //#ifdef GRABBER_COMPATIBLE