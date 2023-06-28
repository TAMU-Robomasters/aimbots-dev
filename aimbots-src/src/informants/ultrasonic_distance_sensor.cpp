#include "utils/robot_specific_inc.hpp"
#ifdef ULTRASONIC

#include "drivers.hpp"
#include "ultrasonic_distance_sensor.hpp"

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
float UltrasonicDistanceSensor::echoStartLeftuS = 0.0f;
float UltrasonicDistanceSensor::echoStartRightuS = 0.0f;
float UltrasonicDistanceSensor::prevDistanceLeft = 0.0f;
float UltrasonicDistanceSensor::prevDistanceRight = 0.0f;
float UltrasonicDistanceSensor::echoEndLeftuS = 0.0f;
float UltrasonicDistanceSensor::echoEndRightuS = 0.0f;
float UltrasonicDistanceSensor::prevEchoEndLeftuS = 0.0f;
float UltrasonicDistanceSensor::prevEchoEndRightuS = 0.0f;
bool UltrasonicDistanceSensor::leftValid = false;
bool UltrasonicDistanceSensor::rightValid = false;
float UltrasonicDistanceSensor::lastReturnedDistance = 0.0f;

tap::arch::PeriodicMilliTimer UltrasonicDistanceSensor::echoTimer(50);
tap::arch::MicroTimeout UltrasonicDistanceSensor::pulseTimer;

float leftDistanceDebug, rightDistanceDebug;
bool leftValidDebug, rightValidDebug;

void UltrasonicDistanceSensor::handleLeftEchoEnd(bool isRising) {
    if (isRising) {
        echoStartLeftuS = tap::arch::clock::getTimeMicroseconds();
    } else {
        // Check how long it's been since we sent the trigger pulse and find the distance from that
        prevEchoEndLeftuS = echoEndLeftuS;
        echoEndLeftuS = tap::arch::clock::getTimeMicroseconds();

        prevDistanceLeft = distanceLeft;
        distanceLeft = ((echoEndLeftuS - echoStartLeftuS)) * CM_PER_uS + ULTRASONIC_OFFSET;

        bool timeValid = (echoEndLeftuS - echoStartLeftuS) < TIMEOUT_DURATION;
        bool rangeValid = distanceLeft >= ULTRASONIC_MIN_VALID_RANGE && distanceLeft <= ULTRASONIC_MAX_VALID_RANGE;
        bool velocityValid = abs((distanceLeft - prevDistanceLeft) / (echoEndLeftuS - prevEchoEndLeftuS)) * 1000000 < ULTRASONIC_MAX_VALID_SPEED;
        leftValid = timeValid && rangeValid && velocityValid;

        leftDistanceDebug = distanceLeft;
        leftValidDebug = leftValid;
    }
}

void UltrasonicDistanceSensor::handleRightEchoEnd(bool isRising) {
    if (isRising) {
        echoStartRightuS = tap::arch::clock::getTimeMicroseconds();
    } else {
        // Check how long it's been since we sent the trigger pulse and find the distance from that
        prevEchoEndRightuS = echoEndRightuS;
        echoEndRightuS = tap::arch::clock::getTimeMicroseconds();

        prevDistanceRight = distanceRight;
        distanceRight = ((echoEndRightuS - echoStartRightuS)) * CM_PER_uS + ULTRASONIC_OFFSET;

        bool timeValid = (echoEndRightuS - echoStartRightuS) < TIMEOUT_DURATION;
        bool rangeValid = distanceRight >= ULTRASONIC_MIN_VALID_RANGE && distanceRight <= ULTRASONIC_MAX_VALID_RANGE;
        bool velocityValid = abs((distanceRight - prevDistanceRight) / (echoEndRightuS - prevEchoEndRightuS)) * 1000000 < ULTRASONIC_MAX_VALID_SPEED;
        rightValid = timeValid && rangeValid && velocityValid;

        rightDistanceDebug = distanceRight;
        rightValidDebug = rightValid;
    }
}

UltrasonicDistanceSensor::UltrasonicDistanceSensor(src::Drivers* drivers)
    : drivers(drivers) {}

// initialize
void UltrasonicDistanceSensor::initialize() {
    LeftEchoPin::setInput(modm::platform::Gpio::InputType::PullDown);
    LeftEchoPin::enableExternalInterruptVector(0);
    LeftEchoPin::enableExternalInterrupt();
    LeftEchoPin::setInputTrigger(modm::platform::Gpio::InputTrigger::BothEdges);

    RightEchoPin::setInput(modm::platform::Gpio::InputType::PullDown);
    RightEchoPin::enableExternalInterruptVector(0);
    RightEchoPin::enableExternalInterrupt();
    RightEchoPin::setInputTrigger(modm::platform::Gpio::InputTrigger::BothEdges);
}

void UltrasonicDistanceSensor::update() {
    // Send 10 uS pulse every 50ms or so
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

float UltrasonicDistanceSensor::getRailPosition() {
    if (leftValid && rightValid) {
        lastReturnedDistance = (getLeftDistance() + getRightDistance()) / 2.0;
    } else if (leftValid) {
        lastReturnedDistance = getLeftDistance();
    } else if (rightValid) {
        lastReturnedDistance = getRightDistance();
    }  // else ur done for

    return lastReturnedDistance;
}

float UltrasonicDistanceSensor::getLeftDistance() {
    if (ORIGIN_SIDE == LEFT) {
        return distanceLeft + ULTRASONIC_LENGTH / 2.0;
    } else
        return USABLE_RAIL_LENGTH * 100 - distanceLeft - (ULTRASONIC_LENGTH / 2.0);
}

float UltrasonicDistanceSensor::getRightDistance() {
    if (ORIGIN_SIDE == RIGHT) {
        return distanceRight + ULTRASONIC_LENGTH / 2.0;
    } else
        return USABLE_RAIL_LENGTH * 100 - distanceRight - (ULTRASONIC_LENGTH / 2.0);
}

}  // namespace src::Informants

#endif // ULTRASONIC