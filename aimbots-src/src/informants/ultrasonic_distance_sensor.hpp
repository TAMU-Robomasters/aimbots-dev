#pragma once

#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "utils/common_types.hpp"

namespace src {
class Drivers;
}

namespace src::Informants {

class UltrasonicDistanceSensor {
   public:
    using LeftEchoPin = modm::platform::GpioC6;    // using port C5, echo for C5 is gpio_C6
    using RightEchoPin = modm::platform::GpioE13;  // using port C3, echo for C3 is gpio_E13

    static void handleLeftEchoEnd(bool isRising);
    static void handleRightEchoEnd(bool isRising);

    UltrasonicDistanceSensor(src::Drivers* drivers);

    void initialize();
    void update();

   private:
    src::Drivers* drivers;

    static float distanceLeft;
    static float distanceRight;
    static float echoStartLeftMS;
    static float echoStartRightMS;

    static tap::arch::PeriodicMilliTimer echoTimer;
    static tap::arch::MicroTimeout pulseTimer;

    static constexpr tap::gpio::Digital::OutputPin LEFT_TRIGGER_PIN = tap::gpio::Digital::OutputPin::C2;
    static constexpr tap::gpio::Digital::OutputPin RIGHT_TRIGGER_PIN = tap::gpio::Digital::OutputPin::C4;
    static constexpr uint16_t CM_PER_uS = 1 / 58.0f;
};

}  // namespace src::Informants