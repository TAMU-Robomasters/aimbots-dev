#include "ultrasonic_distance_sensor.hpp"

#include "drivers.hpp"

// FIXME: I have not set up the project.xml for this, so we have
//        no idea if these interrupts are used by something else

MODM_ISR(EXTI9_5) {  // using GPIO_C6, EXTI9_5_IRQn referenced as the "ExternalInterruptIRQ" in gpio_C6.hpp
    utils::UltrasonicDistanceSensor::LeftEchoTriggerPin::acknowledgeExternalInterruptFlag();
    utils::UltrasonicDistanceSensor::handleLeftEchoEnd();
}

static bool hitRightRisingEdge = false;
MODM_ISR(EXTI15_10) {
    utils::UltrasonicDistanceSensor::RightEchoTriggerPin::acknowledgeExternalInterruptFlag();
    utils::UltrasonicDistanceSensor::handleRightEchoEnd();
}

namespace utils {

float UltrasonicDistanceSensor::distanceLeft = 0.0f;
float UltrasonicDistanceSensor::distanceRight = 0.0f;
tap::arch::PeriodicMilliTimer UltrasonicDistanceSensor::echoTimer(50);
tap::arch::Timeout UltrasonicDistanceSensor::pulseTimer;

void UltrasonicDistanceSensor::handleLeftEchoEnd() {
    // Check how long it's been since we sent the trigger pulse and find the distance from that
    float echoFinishTime = tap::arch::clock::getTimeMilliseconds();
    distanceLeft = ((echoFinishTime - echoStartTimeMS) * 1000) * CM_PER_uS;
}

void UltrasonicDistanceSensor::handleRightEchoEnd() {
    // Check how long it's been since we sent the trigger pulse and find the distance from that
    float echoFinishTime = tap::arch::clock::getTimeMilliseconds();
    distanceRight = ((echoFinishTime - echoStartTimeMS) * 1000) * CM_PER_uS;
}

UltrasonicDistanceSensor::UltrasonicDistanceSensor(src::Drivers* drivers, uint16_t txPin, uint16_t rxPin) {}

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
    // the echo signal has gone high, but hasn't finished yet.

    if (echoTimer.execute()) {
        LeftEchoTriggerPin::setOutput(true);
        RightEchoTriggerPin::setOutput(true);
        pulseTimer.restart(1);
    }
    if (pulseTimer.execute()) {
        LeftEchoTriggerPin::setOutput(false);
        RightEchoTriggerPin::setOutput(false);
        echoStartTimeMS = tap::arch::clock::getTimeMilliseconds();
    }
}

}  // namespace utils