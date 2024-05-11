#pragma once

#include "tap/control/subsystem.hpp"

#include "supper_cap_protocol.hpp"

#include "drivers.hpp"

namespace src::Informants::SupperCap {

class SupperCapSubsystem : public tap::control::Subsystem {
public:
    SupperCapSubsystem(src::Drivers* drivers);

    mockable void initialize() override;
    mockable void refresh() override;

    inline void setCommand(SupperCapCommand command) { this->currentCommand = command; }

    inline SupperCapMessageRecieved getLastMessage() const { return lastMessage; }

private:
    src::Drivers* drivers;
    SupperCapCommand currentCommand;

    float voltage;
    float power;
    float percent;
    float inputPower;

    SupperCapMessageRecieved lastMessage;
    // commands
};

}  // namespace src::SupperCap