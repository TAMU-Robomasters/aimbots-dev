#include "subsystems/hopper/hopper.hpp"

#include "utils/common_types.hpp"

namespace src::Hopper {

    HopperSubsystem::HopperSubsystem(tap::Drivers * drivers) : Subsystem(drivers),
                                                               hopperMotor(drivers, HOPPER_PIN, 0.0f, 1.0f, 0.001f),
                                                               hopper_state(2) {}

    void HopperSubsystem::initialize() {}

    void HopperSubsystem::refresh() {
        // hopperMotor.setTargetPwm(90.0f);
        hopperMotor.updateSendPwmRamp();
    }

    float hopperAngleSetDisplay = 0.0f;

    void HopperSubsystem::setHopperAngle(float desiredAngle) {
        desiredAngle = 90.0f;
        desiredAngle = tap::algorithms::limitVal<float>(desiredAngle, 0.0, 360.0) / 360.0;  //map 0-360 to 0-1
        hopperAngleSetDisplay = desiredAngle;
        hopperMotor.setTargetPwm(desiredAngle);
        actionStartTime = tap::arch::clock::getTimeMilliseconds();
    }

    bool HopperSubsystem::isHopperReady() const {
        // return (hopperMotor.isRampTargetMet() &&
        //         tap::arch::clock::getTimeMilliseconds() - actionStartTime > HOPPER_MIN_ACTION_DELAY);
        return true;
        //the delay is mostly just to keep commands from ending b4 they should, bc isRampTargetMet() is based on pwm ramp finishing
    }

    uint8_t HopperSubsystem::getHopperState() const {
        return hopper_state;
    }

    void HopperSubsystem::setHopperState(uint8_t new_state) {
        hopper_state = new_state;
    }
};  //namespace src::Hopper