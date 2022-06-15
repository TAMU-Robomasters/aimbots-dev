#include "ultrasonic_distance_sensor.hpp"

#include "drivers.hpp"

// FIXME: I have not set up the project.xml for this, so we have
//        no idea if these interrupts are used by something else

MODM_ISR(EXTI9_5) {  // using GPIO_C6, EXTI9_5_IRQn referenced as the "ExternalInterruptIRQ" in gpio_C6.hpp
    src::Informants::UltrasonicDistanceSensor::LeftEchoPin::acknowledgeExternalInterruptFlag();
    bool isRising = src::Informants::UltrasonicDistanceSensor::LeftEchoPin::read();
    src::Informants::UltrasonicDistanceSensor::handleLeftEchoEnd(isRising);
}

MODM_ISR(EXTI15_10) {
    src::Informants::UltrasonicDistanceSensor::RightEchoPin::acknowledgeExternalInterruptFlag();
    bool isRising = src::Informants::UltrasonicDistanceSensor::RightEchoPin::read();
    src::Informants::UltrasonicDistanceSensor::handleRightEchoEnd(isRising);
}

namespace src::Informants {

float UltrasonicDistanceSensor::distanceLeft = 0.0f;
float UltrasonicDistanceSensor::distanceRight = 0.0f;
float UltrasonicDistanceSensor::echoStartLeftMS = 0.0f;
float UltrasonicDistanceSensor::echoStartRightMS = 0.0f;

tap::arch::PeriodicMilliTimer UltrasonicDistanceSensor::echoTimer(50);
tap::arch::MicroTimeout UltrasonicDistanceSensor::pulseTimer;

float leftDistanceDebug, rightDistanceDebug;
float generalDebug;

void UltrasonicDistanceSensor::handleLeftEchoEnd(bool isRising) {
    if (isRising) {
        echoStartLeftMS = tap::arch::clock::getTimeMicroseconds();
    } else {
        // Check how long it's been since we sent the trigger pulse and find the distance from that
        float echoFinishTime = tap::arch::clock::getTimeMicroseconds();
        distanceLeft = ((echoFinishTime - echoStartLeftMS)) * CM_PER_uS;
        leftDistanceDebug = distanceLeft;
    }
    generalDebug++;
}

void UltrasonicDistanceSensor::handleRightEchoEnd(bool isRising) {
    if (isRising) {
        echoStartRightMS = tap::arch::clock::getTimeMicroseconds();
    } else {
        // Check how long it's been since we sent the trigger pulse and find the distance from that
        float echoFinishTime = tap::arch::clock::getTimeMicroseconds();
        distanceRight = ((echoFinishTime - echoStartRightMS)) * CM_PER_uS;
        rightDistanceDebug = distanceRight;
    }
    generalDebug++;
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

bool readDebug;

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
    readDebug = src::Informants::UltrasonicDistanceSensor::RightEchoPin::read();
}

}  // namespace src::Informants