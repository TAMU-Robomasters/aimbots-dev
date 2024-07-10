#pragma once

#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"

namespace tap {
class Drivers;
}

namespace src::Utils::ClientDisplay {

class ClientDisplaySubsystem : public tap::control::Subsystem {
public:
    ClientDisplaySubsystem(tap::Drivers* drivers);
    virtual ~ClientDisplaySubsystem(){};

    void refreshSafeDisconnect() override {}

    const char* getName() const override { return "Client Display"; }
};

}  // namespace src::Utils::ClientDisplay