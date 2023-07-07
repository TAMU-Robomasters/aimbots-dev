#include "sentry_chassis_auto_nav_evade_command.hpp"

namespace src::Chassis {

SentryChassisAutoNavEvadeCommand(src::Drivers* drivers, ChassisSubsystem* chassis) : drivers(drivers), chassis(chassis) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void SentryChassisAutoNavEvadeCommand::initialize() {}

float angle = 0;

void SentryChassisAutoNavEvadeCommand::execute() {
    // get current location (CONST) (don't know how to do this)
    // angle = toRadians(0)
    // modm:Location2D<float> circleLocation(<currLocation.X + cos(angle)>, <currLocation.Y + sin(angle)>)
    // increment angle by X amount

    // This should make the robot spiral in a helix outward until it reaches a 1 meter radius at which point it will continue
    // to orbit in a circle. This is infinite until command is descheduled and waypoint issued
}

bool SentryChassisAutoNavEvadeCommand::isReady() { return true; }

bool SentryChassisAutoNavEvadeCommand::isFinished() const { return false; }

void SentryChassisAutoNavEvadeCommand::end(bool interrupted) { UNUSED(interrupted); }

}  // namespace src::Chassis