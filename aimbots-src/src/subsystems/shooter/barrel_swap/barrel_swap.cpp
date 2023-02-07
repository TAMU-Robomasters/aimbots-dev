
#ifdef TARGET_STANDARD

#include "barrel_swap.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Shooter {
BarrelSwapSubsytem::BarrelSwapSubsytem(tap::Drivers*) : Subsystem(drivers) {}

BarrelSwapSubsytem::BarrelSwapSubsytem(src::Drivers* drivers, ShooterSubsystem* shooter) {
    this->drivers = drivers;
    this->shooter = shooter;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void BarrelSwapSubsytem::initialize() {
    // move until hits limit switch
    // LimitSwitch.isFalling
    // reset encoders (how)
}

void BarrelSwapSubsytem::refresh() {
    
    if (shooter.runShooterCommand.isFinished) {
        clock_t t = clock();
        // if (clock() - t <(0.2*CLOCKS_PER_SEC) && current shooter heat > max_heat) {
            // current shooter.stopShooterCommand.execute();
            // swap shooters
            // 
            // current shooter = other shooter
        }
    }
    
}  // namespace src::Shooter

#endif