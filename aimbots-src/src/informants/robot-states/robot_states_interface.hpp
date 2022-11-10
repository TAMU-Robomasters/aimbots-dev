#pragma once

#include "src/drivers.hpp"
#include "utils/common_types.hpp"

#include "robot_state.hpp"

namespace src::robotStates {
class RobotStates {
private:
    Matrix<Robot, 2, 9> robotStates;
    src::Drivers* drivers;

public:
    RobotStates(src::Drivers* drivers);
    ~RobotStates();

    void setIndivualRobotState(Robot robot);
    Robot getIndivualRobotState(int robotNumber, Team teamColor);

    void updateRobotState(int robotNumber, Team teamColor, float x, float y, float z, int health);
    void updateRobotStateHealth(int robotNumber, Team teamColor, int health);
    void updateRobotStatePosition(int robotNumber, Team teamColor, float x, float y, float z);

    void updateRefSystem();
};
}  // namespace src::robotStates