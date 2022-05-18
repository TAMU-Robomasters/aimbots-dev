#include "drivers.hpp"
#include "subsystems/hopper/hopper.hpp"
#include "tap/communication/gpio/pwm.hpp"

#include "utils/common_types.hpp"

namespace src::Hopper {

    HopperSubsystem::HopperSubsystem(tap::Drivers * drivers) : Subsystem(drivers),
                                                               drivers(drivers),
                                                               hopperMotor(drivers, HOPPER_PIN, 0.85, 0.1325f, 0.01f),
                                                               hopper_state(2) {}

    void HopperSubsystem::initialize() {
        drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 330);  // Timer 1 for C1 Pin
    }

    float testVal = 0.85f;
    // minimum pwm write value is 0.1325f
    // maximum pwm write value is 0.85f

    void HopperSubsystem::refresh() {
        // testVal = fmod(testVal + 0.0001f, 1.0f);
        // hopperMotor.setTargetPwm(testVal);
        hopperMotor.updateSendPwmRamp();
        // drivers->pwm.write(testVal, HOPPER_PIN);
    }

    float hopperAngleSetDisplay = 0.0f;

    void HopperSubsystem::setHopperAngle(float desiredAngle) {
        desiredAngle = tap::algorithms::limitVal<float>(desiredAngle, 0.0f, 270.0f) / 270.0f;  //map 0-360 to 0-1
        hopperAngleSetDisplay = desiredAngle;
        hopperMotor.setTargetPwm(desiredAngle);
        actionStartTime = tap::arch::clock::getTimeMilliseconds();
    }

    bool HopperSubsystem::isHopperReady() const {
        // return (hopperMotor.isRampTargetMet() && (tap::arch::clock::getTimeMilliseconds() - actionStartTime) > HOPPER_MIN_ACTION_DELAY);
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