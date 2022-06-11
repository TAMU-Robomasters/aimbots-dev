#include "ultrasonic_distance_sensor.hpp"

#include "drivers.hpp"

// FIXME: I have not set up the project.xml for this, so we have
//        no idea if these interrupts are used by something else

MODM_ISR(EXTI9_5) {  // using GPIO_C6, EXTI9_5_IRQn referenced as the "ExternalInterruptIRQ" in gpio_C6.hpp
    utils::UltrasonicDistanceSensor::LeftEchoPin::acknowledgeExternalInterruptFlag();
    bool isRising = utils::UltrasonicDistanceSensor::LeftEchoPin::read();
    utils::UltrasonicDistanceSensor::handleLeftEchoEnd(isRising);
}

MODM_ISR(EXTI15_10) {
    utils::UltrasonicDistanceSensor::RightEchoPin::acknowledgeExternalInterruptFlag();
    bool isRising = utils::UltrasonicDistanceSensor::RightEchoPin::read();
    utils::UltrasonicDistanceSensor::handleRightEchoEnd(isRising);
}

namespace utils {

float UltrasonicDistanceSensor::distanceLeft = 0.0f;
float UltrasonicDistanceSensor::distanceRight = 0.0f;
float UltrasonicDistanceSensor::echoStartLeftMS = 0.0f;
float UltrasonicDistanceSensor::echoStartRightMS = 0.0f;

tap::arch::PeriodicMilliTimer UltrasonicDistanceSensor::echoTimer(50);
tap::arch::MicroTimeout UltrasonicDistanceSensor::pulseTimer;

void UltrasonicDistanceSensor::handleLeftEchoEnd(bool isRising) {
    if (isRising) {
        echoStartLeftMS = tap::arch::clock::getTimeMilliseconds();
    } else {
        // Check how long it's been since we sent the trigger pulse and find the distance from that
        float echoFinishTime = tap::arch::clock::getTimeMilliseconds();
        distanceLeft = ((echoFinishTime - echoStartLeftMS) * 1000) * CM_PER_uS;
    }
}

void UltrasonicDistanceSensor::handleRightEchoEnd(bool isRising) {
    if (isRising) {
        echoStartRightMS = tap::arch::clock::getTimeMilliseconds();
    } else {
        // Check how long it's been since we sent the trigger pulse and find the distance from that
        float echoFinishTime = tap::arch::clock::getTimeMilliseconds();
        distanceRight = ((echoFinishTime - echoStartRightMS) * 1000) * CM_PER_uS;
    }
}

UltrasonicDistanceSensor::UltrasonicDistanceSensor(src::Drivers* drivers)
    : drivers(drivers) {}

void UltrasonicDistanceSensor::initialize() {
    LeftEchoPin::setInput(modm::platform::Gpio::InputType::PullDown);
    LeftEchoPin::enableExternalInterruptVector(0);
    LeftEchoPin::enableExternalInterrupt();
    LeftEchoPin::setInputTrigger(modm::platform::Gpio::InputTrigger::BothEdges);  // I think both edge trigger should be more accurate

    RightEchoPin::setInput(modm::platform::Gpio::InputType::PullDown);
    RightEchoPin::enableExternalInterruptVector(0);
    RightEchoPin::enableExternalInterrupt();
    RightEchoPin::setInputTrigger(modm::platform::Gpio::InputTrigger::BothEdges);
}

void UltrasonicDistanceSensor::update() {
    // Send pulse, start timer, and turn of pulse to see how long it takes
    // until we get the end of the pulse signal.

    if (echoTimer.execute()) {
        drivers->digital.set(LEFT_TRIGGER_PIN, true);
        drivers->digital.set(RIGHT_TRIGGER_PIN, true);
        pulseTimer.restart(10);  // 10 microsecond pulse
    }

    if (pulseTimer.execute()) {
        drivers->digital.set(LEFT_TRIGGER_PIN, false);
        drivers->digital.set(RIGHT_TRIGGER_PIN, false);
    }
}

}  // namespace utils