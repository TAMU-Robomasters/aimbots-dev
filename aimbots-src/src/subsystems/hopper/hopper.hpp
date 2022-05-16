#pragma once

#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "robots/standard/standard_constants.hpp"

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

    //motor control commands here, specific open/close commands separate?
    //need to know more about servo class
    
    //taproot takes 0-1 which I'm assuming maps to 0-360 degrees
    //want to take 0-360 here and map to taproot 0-1

    /**
     * @brief do not call continuously!1!
     * 
     */
    void setHopperAngle(float desiredAngle);

    /**
     * @brief rough estimate for if servo movement finished
     * 
     * @return true if pwm ramp finished
     * @return false 
     */
    bool isHopperReady() const;

    /**
     * @brief returns if hopper is open/closed/unknown
     * 
     * @return uint8_t 
     */
    uint8_t getHopperState() const;

    /**
     * @brief setter for hopper state integer
     * 
     */
    void setHopperState(uint8_t new_state);

    private:
    Servo hopper;
    uint8_t hopper_state;
    uint32_t actionStartTime; //milliseconds
};
}; //namespace src::Hopper