#include "subsystems/hopper/hopper.hpp"

#include "utils/common_types.hpp"

namespace src::Hopper {

HopperSubsystem::HopperSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
    hopper(drivers, HOPPER_PIN,HOPPER_PWM_MAX,HOPPER_PWM_MIN,HOPPER_MAX_ACCELERATION)
    //TODO: figure out what pin this should be (this is an arbitrary pin)
    //TODO: setup servo settings in standard constants 
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
}

bool HopperSubsystem::isHopperReady() const {
    return hopper.isRampTargetMet();
}

}; //namespace src::Hopper