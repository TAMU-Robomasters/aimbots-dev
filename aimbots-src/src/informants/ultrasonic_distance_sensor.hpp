#pragma once

#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "utils/common_types.hpp"

// sentry ultrasonic settings
static constexpr bool ORIGIN_SIDE = false;                   // false is 'left' origin, true is 'right' origin... still need to make sure left/right are actually left/right but should work regardless
static constexpr int TIMEOUT_DURATION = 30000;               // microseconds
static constexpr float ULTRASONIC_MAX_VALID_SPEED = 20.0f;   // cm/s, robot speed above which ultrasonic is ignored
static constexpr float ULTRASONIC_LENGTH = 39.0f;            // cm, distance between the two ultrasonics (PCB to PCB)
static constexpr float ULTRASONIC_OFFSET = 0.5f;             // cm, "distance from PCB to outer plate"
static constexpr float ULTRASONIC_MIN_VALID_RANGE = 10.0f;   // cm
static constexpr float ULTRASONIC_MAX_VALID_RANGE = 250.0f;  // cm
static constexpr float ULTRASONIC_TRUST_FACTOR = 0.5f;       // range 0-1

enum leftAndRight {
    LEFT = 0,
    RIGHT = 1
};

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

    /**
     * @brief fancy getter that returns position of ultrasonic from origin (in cm)
     */
    static float getLeftDistance();

    /**
     * @brief returns position of ultrasonic from origin (in cm)
     */
    static float getRightDistance();

    /**
     * @brief Returns rail position in cm. The origin can be on either side of the rail (see bool in sentry_constants.hpp).
     * Depends on total rail length (in sentry_constants.hpp)
     *
     * @return float rail position in cm
     */
    static float getRailPosition();

    /**
     * @brief Returns bool to indicate if either ultrasonic is reading-- if not, any data provided by ultrasonic class is outdated
     */
    static inline bool isDataValid() { return (leftValid && rightValid); };

   private:
    src::Drivers* drivers;

    static float echoStartLeftuS;
    static float echoStartRightuS;

    static float distanceLeft;
    static float distanceRight;
    static float prevDistanceLeft;
    static float prevDistanceRight;

    static float echoEndLeftuS;
    static float echoEndRightuS;
    static float prevEchoEndLeftuS;
    static float prevEchoEndRightuS;

    static bool leftValid;
    static bool rightValid;

    static float lastReturnedDistance;

    static tap::arch::PeriodicMilliTimer echoTimer;
    static tap::arch::MicroTimeout pulseTimer;

    static constexpr tap::gpio::Digital::OutputPin LEFT_TRIGGER_PIN = tap::gpio::Digital::OutputPin::C2;
    static constexpr tap::gpio::Digital::OutputPin RIGHT_TRIGGER_PIN = tap::gpio::Digital::OutputPin::C4;
    static constexpr float CM_PER_uS = 1 / 58.0f;
    static constexpr float TIMEOUT_DURATION_uS = 30000;
};

}  // namespace src::Informants