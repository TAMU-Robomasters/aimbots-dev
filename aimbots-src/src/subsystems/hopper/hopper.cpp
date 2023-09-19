#include "subsystems/hopper/hopper.hpp"

#include "tap/communication/gpio/pwm.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef HOPPER_LID_COMPATIBLE

namespace src::Hopper {

HopperSubsystem::HopperSubsystem(
    tap::Drivers* drivers)
    : Subsystem(drivers),
      drivers(drivers),
      hopperMotor(drivers, HOPPER_PIN, HOPPER_MAX_PWM, HOPPER_MIN_PWM, HOPPER_PWM_RAMP_SPEED),
      hopperState(CLOSED) {}

void HopperSubsystem::initialize() {
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 330);  // Timer 1 for C1 Pin
}

void HopperSubsystem::refresh() { hopperMotor.updateSendPwmRamp(); }

float hopperAngleSetDisplay = 0.0f;

void HopperSubsystem::setHopperAngle(float desiredAngle) {
    desiredAngle = tap::algorithms::limitVal<float>(
        desiredAngle,
        HOPPER_MIN_ANGLE,
        HOPPER_MAX_ANGLE);  // Limit inputs to min/max of motor
    hopperAngleSetDisplay = desiredAngle;
    hopperMotor.setTargetPwm(REMAP(desiredAngle, HOPPER_MIN_ANGLE, HOPPER_MAX_ANGLE, HOPPER_MIN_PWM, HOPPER_MAX_PWM));
    actionStartTime = tap::arch::clock::getTimeMilliseconds();
}

bool HopperSubsystem::isHopperReady() const {
    return (
        hopperMotor.isRampTargetMet() && (tap::arch::clock::getTimeMilliseconds() - actionStartTime) > HOPPER_MIN_ACTION_DELAY);
    // return true;
    // the delay is mostly just to keep commands from ending b4 they should, bc isRampTargetMet() is based on pwm ramp
    // finishing
}

uint8_t state_;
uint8_t HopperSubsystem::getHopperState() const {
    state_ = hopperState;
    return hopperState;
}

uint8_t newStateDisplay;
void HopperSubsystem::setHopperState(uint8_t newState) {
    newStateDisplay = newState;
    hopperState = newState;
}

bool HopperSubsystem::isHopperOpen() const {
    if (hopperState == OPEN) {
        return true;
    } else {
        return false;
    }
}
};  // namespace src::Hopper

#endif  // #ifdef HOPPER_LID_COMPATIBLE