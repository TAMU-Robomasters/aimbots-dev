#pragma once

#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"

namespace src::Hopper {

enum HopperState : uint8_t {
    CLOSED = 0,
    OPEN = 1,
    UNKNOWN = 2
};

class HopperSubsystem : public tap::control::Subsystem {
   public:
    HopperSubsystem(tap::Drivers* drivers);

    mockable void initialize() override;
    void refresh() override;

    // motor control commands here, specific open/close commands separate?
    // need to know more about servo class

    // taproot takes 0-1 which I'm assuming maps to 0-360 degrees
    // want to take 0-360 here and map to taproot 0-1

    /**
     * @brief Sets angle for hopper servo to turn to and maintain (don't call continuously!!!)
     *
     */
    void setHopperAngle(float desiredAngle);

    /**
     * @brief Returns true if hopper PWM ramp is finished AND minimum delay (declared in standard_constants) has passed
     *
     */
    bool isHopperReady() const;

    /**
     * @brief Returns the current state of the hopper as a uint8_t (check HopperState enum)
     *
     */
    uint8_t getHopperState() const;

    /**
     * @brief setter for hopper state integer
     *
     */
    void setHopperState(uint8_t new_state);

   private:
    tap::Drivers* drivers;

    Servo hopperMotor;
    uint8_t hopper_state;
    uint32_t actionStartTime;  // milliseconds
};
};  // namespace src::Hopper