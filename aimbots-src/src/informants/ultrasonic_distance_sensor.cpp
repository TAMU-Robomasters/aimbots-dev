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
float UltrasonicDistanceSensor::echoStartLeftuS = 0.0f;
float UltrasonicDistanceSensor::echoStartRightuS = 0.0f;

bool UltrasonicDistanceSensor::leftTimeoutStatus = false;
bool UltrasonicDistanceSensor::rightTimeoutStatus = false;
float UltrasonicDistanceSensor::lastReturnedDistance = 0.0f;

tap::arch::PeriodicMilliTimer UltrasonicDistanceSensor::echoTimer(50);
tap::arch::MicroTimeout UltrasonicDistanceSensor::pulseTimer;

float leftDistanceDebug, rightDistanceDebug;

void UltrasonicDistanceSensor::handleLeftEchoEnd(bool isRising) {
    if (isRising) {
        echoStartLeftuS = tap::arch::clock::getTimeMicroseconds();
    } else {
        // Check how long it's been since we sent the trigger pulse and find the distance from that
        float echoFinishTime = tap::arch::clock::getTimeMicroseconds();

        leftTimeoutStatus = (echoFinishTime - echoStartLeftuS) > TIMEOUT_DURATION;
        distanceLeft = ((echoFinishTime - echoStartLeftuS)) * CM_PER_uS;
        leftDistanceDebug = distanceLeft;
    }
}

void UltrasonicDistanceSensor::handleRightEchoEnd(bool isRising) {
    if (isRising) {
        echoStartRightuS = tap::arch::clock::getTimeMicroseconds();
    } else {
        // Check how long it's been since we sent the trigger pulse and find the distance from that
        float echoFinishTime = tap::arch::clock::getTimeMicroseconds();

        rightTimeoutStatus = (echoFinishTime - echoStartRightuS) > TIMEOUT_DURATION ;
        distanceRight = ((echoFinishTime - echoStartRightuS)) * CM_PER_uS; //math will not blow up if sensor timed out, but will be erroneous
        rightDistanceDebug = distanceRight;
    }
}

UltrasonicDistanceSensor::UltrasonicDistanceSensor(src::Drivers* drivers)
    : drivers(drivers) {}

//initialize
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
    if(!leftTimeoutStatus && !rightTimeoutStatus) {
        lastReturnedDistance = (getLeftDistance() + getRightDistance()) / 2.0;
    } else if(!leftTimeoutStatus) {
        lastReturnedDistance = getLeftDistance();
    } else if(!rightTimeoutStatus) {
        lastReturnedDistance = getRightDistance();
    } //else ur done for

    return lastReturnedDistance;
}

float UltrasonicDistanceSensor::getLeftDistance() {
    if(ORIGIN_SIDE == LEFT) {
        return distanceLeft + ULTRASONIC_LENGTH / 2.0;
    } else return FULL_RAIL_LENGTH_CM - distanceLeft - ULTRASONIC_LENGTH / 2.0;
}

float UltrasonicDistanceSensor::getRightDistance() {
    if(ORIGIN_SIDE == RIGHT) {
        return distanceRight + ULTRASONIC_LENGTH / 2.0;
    } else return FULL_RAIL_LENGTH_CM - distanceRight - ULTRASONIC_LENGTH / 2.0;
}

}  // namespace src::Informants