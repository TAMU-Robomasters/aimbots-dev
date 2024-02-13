#pragma once

#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"


#ifdef GRABBER_COMPATIBLE

namespace src::Grabber {

class GrabberSubsystem : public tap::control::Subsystem {
public:
    GrabberSubsystem(tap::Drivers* drivers);

    void initialize() override;
    
    void refresh() override;

    void activate();

    void deactivate();

    void unknown();

private:
    tap::gpio::Pwm* const pwm;
};

}

#endif //GRABBER_COMPATIBLE