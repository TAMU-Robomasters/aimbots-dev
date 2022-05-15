#pragma once

#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "robots/standard/standard_constants.hpp"

namespace src::Hopper {

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
     * @brief wrapper for servo's isRampTargetMet.. maybe just make servo public member?
     * 
     * @return true 
     * @return false 
     */
    bool isHopperReady() const;

    private:
    Servo hopper;
};
}; //namespace src::Hopper