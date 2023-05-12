#pragma once

#include "tap/control/subsystem.hpp"

#include "informants/robot-states/robot_state.hpp"

#include "communication_message.hpp"
#include "communication_request_transmiter.hpp"
#include "drivers.hpp"

namespace src::Communication {
class CommunicationRequestSubsystem : public tap::control::Subsystem {
public:
    CommunicationRequestSubsystem(src::Drivers* drivers);

    void refresh() override;

    inline void queueRequest(MessageType type) { robotTransmiter.updateQue(type); }

private:
    CommunicationRequestTransmiter robotTransmiter;
    Drivers* drivers;
};
}  // namespace src::Communication
