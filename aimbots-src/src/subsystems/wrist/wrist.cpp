#include "subsystems/wrist/wrist.hpp"

#ifdef WRIST_COMPATIBLE
namespace src::Wrist {

WristSubsystem::WristSubsystem(src::Drivers* drivers)
    : drivers(drivers),
      Subsystem(drivers) 
     {
    BuildMotors();
     }

void WristSubsystem::initialize() {
    // idk
}

void WristSubsystem::refresh() {}

void WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {}

void WristSubsystem::setArmAngles() {}

void WristSubsystem::setTargetYawAngle() {}

void WristSubsystem::setTargetPitchAngle() {}

void WristSubsystem::setTargetRollAngle() {}
};      // namespace src::Wrist
#endif  // #ifdef WRIST_COMPATIBLE
