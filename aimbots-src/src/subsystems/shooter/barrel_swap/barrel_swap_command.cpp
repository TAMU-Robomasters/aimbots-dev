
#ifdef TARGET_STANDARD

#include "barrel_swap_command.hpp"

namespace src::Shooter {

BarrelSwapCommand::BarrelSwapCommand(src::Drivers* drivers, BarrelSwapSubsytem* barrelSwap) : drivers(drivers), barrelSwap(barrelSwap) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(barrelSwap));
}

void BarrelSwapCommand::initialize() {}

void BarrelSwapCommand::end(bool) {
    // turn the shooter command on
}

bool BarrelSwapCommand::isReady() { return true; }

bool BarrelSwapCommand::isFinished() const { return false; }

}  // namespace src::Shooter

#endif
