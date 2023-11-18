#pragma once

#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"


#ifdef GRABBER_COMPATIBLE //TODO: Define this in a constants file later

namespace src::Grabber {

class GrabberSubsystem : public tap::control::Subsystem {
public:
    GrabberSubsystem(tap::Drivers* drivers);

    mockable void initialize() override;
    
    mockable void refresh() override;

    //mockable void deactivate() override;
    mockable void activate();

    mockable void deactivate();

    mockable void unknown();

private:
    tap::Drivers* drivers;

};

} //namespace src::Grabber


#endif //GRABBER_COMPATIBLE