#pragma once

#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

namespace src::utils::display {

class ClientDisplaySubsystem : public tap::control::Subsystem {
public:
    ClientDisplaySubsystem(tap::Drivers* drivers);
    virtual ~ClientDisplaySubsystem(){};

    // void refresh() override;

    const char* getName() override { return "client display"; }
};
}  // namespace src::utils::display