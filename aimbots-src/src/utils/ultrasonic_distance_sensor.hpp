#pragma once

#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "utils/common_types.hpp"

namespace src {
class Drivers;
}

namespace utils {

class UltrasonicDistanceSensor {
   public:
    using LeftEchoTriggerPin = modm::platform::GpioC6;    // using port C5, echo for C5 is gpio_C6
    using RightEchoTriggerPin = modm::platform::GpioE13;  // using port C3, echo for C3 is gpio_E13

    static void handleLeftEchoEnd();
    static void handleRightEchoEnd();

    UltrasonicDistanceSensor(src::Drivers* drivers, uint16_t txPin, uint16_t rxPin);

    void initialize();
    void update();

   private:
    static float distanceLeft;
    static float distanceRight;
    static float echoStartTimeMS;
    static tap::arch::PeriodicMilliTimer echoTimer;
    static tap::arch::Timeout pulseTimer;

    static constexpr uint16_t CM_PER_uS = 1 / 58.0f;
};

}  // namespace utils