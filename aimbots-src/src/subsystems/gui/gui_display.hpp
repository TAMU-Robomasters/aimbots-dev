#pragma once

#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"
#include "drivers.hpp"

namespace src::GUI {

class GUI_DisplaySubsystem : public tap::control::Subsystem {

    //Virtual sub-system, exists only to run commands on it.
    public:
    GUI_DisplaySubsystem(src::Drivers* drivers);
};

};