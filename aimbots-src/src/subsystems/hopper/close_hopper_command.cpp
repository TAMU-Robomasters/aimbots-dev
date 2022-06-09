#ifdef TARGET_STANDARD

#include "subsystems/hopper/close_hopper_command.hpp"

namespace src::Hopper {
CloseHopperCommand::CloseHopperCommand(src::Drivers* drivers, HopperSubsystem* hopper) {
    this->drivers = drivers;
    this->hopper = hopper;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(hopper));
}

void CloseHopperCommand::initialize() {
    hopper->setHopperAngle(HOPPER_CLOSED_ANGLE);
}

void CloseHopperCommand::execute() {
}

void CloseHopperCommand::end(bool) {
}

bool CloseHopperCommand::isReady() {
    return true;
}

bool CloseHopperCommand::isFinished() const {
    return hopper->isHopperReady();
}

};  // namespace src::Hopper
#endif