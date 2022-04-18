#include "ultrasonic_distance_sensor.hpp"

#include "drivers.hpp"

// FIXME: I have not set up the project.xml for this, so we have
//        no idea if these interrupts are used by something else

MODM_ISR(EXTI0_IRQn) {
    utils::UltrasonicDistanceSensor::LeftEchoTriggerPin::acknowledgeExternalInterruptFlag();
    utils::UltrasonicDistanceSensor::handleLeftEchoEnd();
}

static bool hitRightRisingEdge = false;
MODM_ISR(EXTI1_IRQn) {
    utils::UltrasonicDistanceSensor::RightEchoTriggerPin::acknowledgeExternalInterruptFlag();
    utils::UltrasonicDistanceSensor::handleRightEchoEnd();
}

namespace utils {

float UltrasonicDistanceSensor::distanceLeft = 0.0f;
float UltrasonicDistanceSensor::distanceRight = 0.0f;
tap::arch::PeriodicMilliTimer UltrasonicDistanceSensor::echoTimer;

void UltrasonicDistanceSensor::handleLeftEchoEnd() {
    // Check how long it's been since we sent the trigger pulse and find the distance from that
}

void UltrasonicDistanceSensor::handleRightEchoEnd() {
    // Check how long it's been since we sent the trigger pulse and find the distance from that
}

UltrasonicDistanceSensor::UltrasonicDistanceSensor(src::Drivers* drivers, uint16_t txPin, uint16_t rxPin) { }

void UltrasonicDistanceSensor::initialize() {
    LeftEchoTriggerPin::setInput(modm::platform::Gpio::InputType::PullDown);
    LeftEchoTriggerPin::enableExternalInterruptVector(0);
    LeftEchoTriggerPin::enableExternalInterrupt();
    LeftEchoTriggerPin::setInputTrigger(modm::platform::Gpio::InputTrigger::FallingEdge);

    RightEchoTriggerPin::setInput(modm::platform::Gpio::InputType::PullDown);
    RightEchoTriggerPin::enableExternalInterruptVector(0);
    RightEchoTriggerPin::enableExternalInterrupt();
    RightEchoTriggerPin::setInputTrigger(modm::platform::Gpio::InputTrigger::FallingEdge);
}

void UltrasonicDistanceSensor::update() {
    // Send pulse, start timer, and turn of pulse to see how long it takes
    // until we get the end of the pulse signal.

    // We're just gonna assume that the instant we set the pin high, that
    // the echo signal has already come back, but hasn't finished yet.
}

}