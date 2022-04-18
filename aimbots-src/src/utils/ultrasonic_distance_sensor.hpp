#pragma once

#include "tap/communication/gpio/pwm.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/architecture/periodic_timer.hpp"

#include "utils/common_types.hpp"

namespace src { class Drivers; }

namespace utils {

class UltrasonicDistanceSensor {
   public:
    using LeftEchoTriggerPin = modm::platform::GpioI0;
    using RightEchoTriggerPin = modm::platform::GpioI1;

    static void handleLeftEchoEnd();
    static void handleRightEchoEnd();

    UltrasonicDistanceSensor(src::Drivers* drivers, uint16_t txPin, uint16_t rxPin);

    void initialize();
    void update();

   private:
    static float distanceLeft;
    static float distanceRight;
    static tap::arch::PeriodicMilliTimer echoTimer;

    static constexpr uint16_t CM_PER_uS = 1 / 58.0f;
};

}