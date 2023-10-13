#include "subsystems/wrist/wrist.hpp"


#ifdef WRIST_COMPATIBLE
namespace src::Wrist {

WristSubsystem::WristSubsystem(src::Drivers* drivers)
    : drivers(drivers),
        Subsystem(drivers) {} //,
    //   yawMotor(drivers,0,0 /*YAW_MOTOR_ID, WRIST_BUS*/, true, "Yaw Motor"),
    //   rollMotor(drivers, 0,0  /*ROLL_MOTOR_ID, WRIST_BUS*/, true, "Roll Motor"),
    //   pitchMotor(drivers, 0,0 /*PITCH_MOTOR_ID, WRIST_BUS*/, true, "Pitch Motor") {}

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
