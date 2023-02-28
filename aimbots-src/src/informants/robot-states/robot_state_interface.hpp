#pragma once

#include "informants/communication/communication_message.hpp"
#include "utils/common_types.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
using namespace src::Communication;

namespace src::RobotStates {
class RobotStates {
private:
    Matrix<Robot, 2, 9> robotStates;
    src::Drivers* drivers;

public:
    RobotStates(src::Drivers* drivers);
    ~RobotStates();

    void setIndivualRobotState(Robot robot);
    Robot getIndivualRobotState(int robotNumber, Team teamColor);

    void updateRobotState(int robotNumber, Team teamColor, short x, short y, short z, int health);
    void updateRobotStateHealth(int robotNumber, Team teamColor, int health);
    void updateRobotStatePosition(int robotNumber, Team teamColor, short x, short y, short z);

    #ifdef TARGET_SENTRY
    robot_state_message_team createMessage();
    #else
    robot_state_message createMesssage();
    robot_state_message_enemy createMessageEnemy();
    #endif
};
void updateRobotStateHero();
void updateRobotStateStandard();
void updateRobotStateSentry();
}  // namespace src::RobotStates