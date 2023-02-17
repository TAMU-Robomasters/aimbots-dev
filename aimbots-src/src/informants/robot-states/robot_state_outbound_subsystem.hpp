#pragma once

#include "tap/control/subsystem.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
#include "robot_state_outbound_transmiter.hpp"
#include "robot_state_message.hpp"


namespace src::robotStates {
class RobotStateOutBoundSubsystem : public tap::control::Subsystem {
public:
    RobotStateOutBoundSubsystem(src::Drivers* drivers);

    void refresh() override;

    inline void queueRequest(MessageType type) { robotTransmiter.updateQue(type); }

private:
    RobotStateOutBoundTransmiter robotTransmiter;
    Drivers* drivers;
};
}  // namespace src::robotState
