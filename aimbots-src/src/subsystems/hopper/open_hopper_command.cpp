#include "subsystems/hopper/open_hopper_command.hpp"

namespace src::Hopper {
OpenHopperCommand::OpenHopperCommand(src::Drivers* drivers, HopperSubsystem* hopper) {
    this->drivers = drivers;
    this->hopper = hopper;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(hopper));
}

void OpenHopperCommand::initialize() { hopper->setHopperAngle(HOPPER_OPEN_ANGLE); }

void OpenHopperCommand::execute() {}

void OpenHopperCommand::end(bool) {}

bool OpenHopperCommand::isReady() { return true; }

bool OpenHopperCommand::isFinished() const { return hopper->isHopperReady(); }

};  // namespace src::Hopper