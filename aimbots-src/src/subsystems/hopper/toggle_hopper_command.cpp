#include "subsystems/hopper/toggle_hopper_command.hpp"

namespace src::Hopper {
ToggleHopperCommand::ToggleHopperCommand(src::Drivers* drivers, HopperSubsystem* hopper, float HOPPER_CLOSED_ANGLE, float HOPPER_OPEN_ANGLE) {
    this->drivers = drivers;
    this->hopper = hopper;
    this->HOPPER_CLOSED_ANGLE = HOPPER_CLOSED_ANGLE;
    this->HOPPER_OPEN_ANGLE = HOPPER_OPEN_ANGLE;
}

void ToggleHopperCommand::initialize() {
    uint8_t state = hopper->getHopperState();
    if (state == UNKNOWN) {  // default 'unknown' action can be open/closed
        hopper->setHopperAngle(HOPPER_CLOSED_ANGLE);
        hopper->setHopperState(CLOSED);
    } else {
        hopper->setHopperAngle(state ? HOPPER_CLOSED_ANGLE : HOPPER_OPEN_ANGLE);
        hopper->setHopperState(!state);
    }
}

void ToggleHopperCommand::execute() {}

void ToggleHopperCommand::end(bool) {}

bool ToggleHopperCommand::isReady() { return true; }

bool ToggleHopperCommand::isFinished() const { return hopper->isHopperReady(); }
};  // namespace src::Hopper