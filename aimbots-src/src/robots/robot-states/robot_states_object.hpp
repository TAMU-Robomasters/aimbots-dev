#pragma once

#include "utils/common_types.hpp"
#include "robot_state.hpp"

namespace src::robotStates {
    class RobotStates{
        private:
            Matrix<Robot, 2, 9> robotStates;

        public:
            RobotStates();
            ~RobotStates();

            void setIndivualRobotState(Robot robot);
            Robot getIndivualRobotState(int robotNumber, Color color);

            void updateRobotState(int robotNumber, Color color, int x, int y, int z, int health);
            void updateRobotStateHealth(int robotNumber, Color color, int health);
            void updateRobotStatePositon(int robotNumber, Color color, int x, int y, int z);

    };
}  // namespace src::robotStates