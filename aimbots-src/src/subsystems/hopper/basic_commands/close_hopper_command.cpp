#include "subsystems/hopper/basic_commands/close_hopper_command.hpp"

#ifdef HOPPER_LID_COMPATIBLE

namespace src::Hopper {
CloseHopperCommand::CloseHopperCommand(src::Drivers* drivers, HopperSubsystem* hopper, float HOPPER_CLOSED_ANGLE) {
    this->drivers = drivers;
    this->hopper = hopper;
    this->HOPPER_CLOSED_ANGLE = HOPPER_CLOSED_ANGLE;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(hopper));
}

void CloseHopperCommand::initialize() {
    hopper->setHopperAngle(HOPPER_CLOSED_ANGLE);
    hopper->setHopperState(CLOSED);
}

void CloseHopperCommand::execute() {}

void CloseHopperCommand::end(bool) {}

bool CloseHopperCommand::isReady() { return true; }

bool CloseHopperCommand::isFinished() const { return hopper->isHopperReady(); }

};  // namespace src::Hopper

#endif  // #ifdef HOPPER_LID_COMPATIBLE