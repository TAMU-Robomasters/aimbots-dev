#include "subsystems/hopper/hopper.hpp"

#include "tap/communication/gpio/pwm.hpp"

#include "utils/common_types.hpp"

#include "drivers.hpp"

namespace src::Hopper {

HopperSubsystem::HopperSubsystem(
    tap::Drivers* drivers,
    tap::gpio::Pwm::Pin hopperPin,
    float hopperMaxPWM,
    float hopperMinPWM,
    float hopperPWMRampSpeed,
    float hopperMinAngle,
    float hopperMaxAngle,
    uint32_t hopperMinActionDelay)
    : Subsystem(drivers),
      drivers(drivers),
      hopperMotor(drivers, hopperPin, hopperMaxPWM, hopperMinPWM, hopperPWMRampSpeed),
      hopper_state(2),
      hopperPin(hopperPin),
      hopperMaxPWM(hopperMaxPWM),
      hopperMinPWM(hopperMinPWM),
      hopperPWMRampSpeed(hopperPWMRampSpeed),
      hopperMinAngle(hopperMinAngle),
      hopperMaxAngle(hopperMaxAngle),
      hopperMinActionDelay(hopperMinActionDelay) {}

void HopperSubsystem::initialize() {
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 330);  // Timer 1 for C1 Pin
}

float testVal = 0.85f;

void HopperSubsystem::refresh() { hopperMotor.updateSendPwmRamp(); }

float hopperAngleSetDisplay = 0.0f;

void HopperSubsystem::setHopperAngle(float desiredAngle) {
    desiredAngle = tap::algorithms::limitVal<float>(
        desiredAngle,
        hopperMinAngle,
        hopperMaxAngle);  // Limit inputs to min/max of motor
    hopperAngleSetDisplay = desiredAngle;
    hopperMotor.setTargetPwm(REMAP(desiredAngle, hopperMinAngle, hopperMaxAngle, hopperMinPWM, hopperMaxPWM));
    actionStartTime = tap::arch::clock::getTimeMilliseconds();
}

bool HopperSubsystem::isHopperReady() const {
    return (
        hopperMotor.isRampTargetMet() && (tap::arch::clock::getTimeMilliseconds() - actionStartTime) > hopperMinActionDelay);
    // return true;
    // the delay is mostly just to keep commands from ending b4 they should, bc isRampTargetMet() is based on pwm ramp
    // finishing
}

uint8_t HopperSubsystem::getHopperState() const { return hopper_state; }

void HopperSubsystem::setHopperState(uint8_t new_state) { hopper_state = new_state; }
};  // namespace src::Hopper