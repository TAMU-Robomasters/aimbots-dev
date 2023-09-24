#include "subsystems/hopper/toggle_hopper_command.hpp"

#ifdef HOPPER_LID_COMPATIBLE

namespace src::Hopper {
ToggleHopperCommand::ToggleHopperCommand(src::Drivers* drivers, HopperSubsystem* hopper, float HOPPER_CLOSED_ANGLE, float HOPPER_OPEN_ANGLE) {
    this->drivers = drivers;
    this->hopper = hopper;
    this->HOPPER_CLOSED_ANGLE = HOPPER_CLOSED_ANGLE;
    this->HOPPER_OPEN_ANGLE = HOPPER_OPEN_ANGLE;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(hopper));
}


bool cPressedDisplay;
bool hopperStateDisplay;
bool commandIsRunning = false;

void ToggleHopperCommand::initialize() {
    hopper->setHopperAngle(hopperClosed ? HOPPER_CLOSED_ANGLE : HOPPER_OPEN_ANGLE);
    commandIsRunning = true;
}

void ToggleHopperCommand::execute() {

    cPressedDisplay = wasCPressed;
    hopperStateDisplay = hopperClosed;

    if (drivers->remote.keyPressed(Remote::Key::C)) {
        wasCPressed = true;
    }

    if (wasCPressed && !drivers->remote.keyPressed(Remote::Key::C)) {
        hopper->setHopperAngle(hopperClosed ? HOPPER_CLOSED_ANGLE : HOPPER_OPEN_ANGLE);
        hopperClosed = !hopperClosed;
        hopper->setHopperState(hopperClosed ? CLOSED : OPEN);
        wasCPressed = false;
    }

}

void ToggleHopperCommand::end(bool) {}

bool ToggleHopperCommand::isReady() { return true; }

bool ToggleHopperCommand::isFinished() const { return false; }
};  // namespace src::Hopper

#endif  // #ifdef HOPPER_LID_COMPATIBLE