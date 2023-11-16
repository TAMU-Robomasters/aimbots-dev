#pragma once

#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "robots/engineer/engineer_constants.hpp"

#ifdef GRABBER_COMPATIBLE //TODO: Define this in a constants file later

namespace src::Grabber {

class GrabberSubsystem : public tap::control::Subsystem {
public:
    GrabberSubsystem(src::Drivers* drivers, tap::gpio::Pwm Pin);

    mockable void initialize() override;
    
    mockable void refresh() override;

    mockable void activate() override;

    mockable void end() override;

private:
    int Pin;
    float duty_circle;
}

} //namespace src::Grabber


#endif //GRABBER_COMPATIBLE