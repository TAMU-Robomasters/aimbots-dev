#include "subsystems/wrist/wrist.hpp"


#ifdef WRIST_COMPATIBLE
namespace src::Wrist {

WristSubsystem::WristSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      yawMotor(drivers, YAW_MOTOR_ID, WRIST_BUS, true, "Yaw Motor"),
      rollMotor(drivers, ROLL_MOTOR_ID, WRIST_BUS, true, "Roll Motor"),
      pitchMotor(drivers, PITCH_MOTOR_ID, WRIST_BUS, true, "Pitch Motor") {}

WristSubsystem::initialize() {
    // idk
}

void WristSubsystem::refresh() {}

void WristSubsystem::calculateArmAngles() {}

void WristSubsystem::setArmAngles() {}

void WristSubsystem::setTargetYawAngle() {}

void WristSubsystem::setTargetPitchAngle() {}

void WristSubsystem::setTargetRollAngle() {}
};      // namespace src::Wrist
#endif  // #ifdef WRIST_COMPATIBLE
