#pragma once

#include "tap/control/subsystem.hpp"

#include "super_cap_protocol.hpp"

#include "drivers.hpp"

namespace src::Informants::SuperCap {

class SuperCapSubsystem : public tap::control::Subsystem {
public:
    SuperCapSubsystem(src::Drivers* drivers);

    mockable void initialize() override;
    mockable void refresh() override;

    inline void setCommand(SuperCapCommand command) { this->currentCommand = command; }

    inline SuperCapMessageRecieved getLastMessage() const { return lastMessage; }

private:
    src::Drivers* drivers;
    SuperCapCommand currentCommand;

    float voltage;
    float power;
    float percent;
    float inputPower;

    SuperCapMessageRecieved lastMessage;
    // commands
};

}  // namespace src::SuperCap