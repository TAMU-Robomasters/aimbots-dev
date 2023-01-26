
#ifdef TARGET_STANDARD

#include "barrel_swap.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Shooter {
BarrelSwapSubsytem::BarrelSwapSubsytem(tap::Drivers*) : Subsystem(drivers) {}

void BarrelSwapSubsytem::initialize() {
    // move until hits limit switch
    // reset encoders
}

void BarrelSwapSubsytem::refresh() {
    // constantly check barrel heat
    // give command to move if 1. not firing for >threshold dead time and 2. barrel is overheating
    // add edge case in case both are overheating (maybe ?)
}
}  // namespace src::Shooter

#endif