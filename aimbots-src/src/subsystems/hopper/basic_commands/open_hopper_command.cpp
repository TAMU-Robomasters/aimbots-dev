#include "subsystems/hopper/basic_commands/open_hopper_command.hpp"

#ifdef HOPPER_LID_COMPATIBLE

namespace src::Hopper {
OpenHopperCommand::OpenHopperCommand(src::Drivers* drivers, HopperSubsystem* hopper, float HOPPER_OPEN_ANGLE) {
    this->drivers = drivers;
    this->hopper = hopper;
    this->HOPPER_OPEN_ANGLE = HOPPER_OPEN_ANGLE;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(hopper));
}

void OpenHopperCommand::initialize() {
    hopper->setHopperAngle(HOPPER_OPEN_ANGLE);
    hopper->setHopperState(OPEN);
}

void OpenHopperCommand::execute() {}

void OpenHopperCommand::end(bool) {}

bool OpenHopperCommand::isReady() { return true; }

bool OpenHopperCommand::isFinished() const { return hopper->isHopperReady(); }

};  // namespace src::Hopper

#endif  // #ifdef HOPPER_LID_COMPATIBLE