#include "subsystems/hopper/hopper.hpp"

#include "utils/common_types.hpp"

namespace src::Hopper {

HopperSubsystem::HopperSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
    hopper(drivers, HOPPER_PIN, HOPPER_MAX_ROTATION/360.0, HOPPER_MIN_ROTATION/360.0, HOPPER_MAX_ACCELERATION),
    hopper_state(2)
    {
    }

void HopperSubsystem::initialize() {
}

void HopperSubsystem::refresh() {
    hopper.updateSendPwmRamp();
}

void HopperSubsystem::setHopperAngle(float desiredAngle) {
    desiredAngle = tap::algorithms::limitVal<float>(desiredAngle,0.0,360.0) / 360.0; //map 0-360 to 0-1
    hopper.setTargetPwm(desiredAngle);
    actionStartTime = tap::arch::clock::getTimeMilliseconds();
}

bool HopperSubsystem::isHopperReady() const {
    return (hopper.isRampTargetMet() && 
    tap::arch::clock::getTimeMilliseconds() - actionStartTime > HOPPER_MIN_ACTION_DELAY);
    //the delay is mostly just to keep commands from ending b4 they should, bc isRampTargetMet() is based on pwm ramp finishing
}

uint8_t HopperSubsystem::getHopperState() const {
    return hopper_state;
}

void HopperSubsystem::setHopperState(uint8_t new_state) {
    hopper_state = new_state;
}

}; //namespace src::Hopper