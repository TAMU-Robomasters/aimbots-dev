#include "robot_states_object.hpp"
#include "robot_state.hpp"

namespace src::robotStates {
RobotStates::RobotStates() {
    robotStates = Matrix<Robot, 2, 9>().zeroMatrix();
    robotStates[Color::RED][0] = Robot(0, 0, 0, 1, 0, Color::RED);  // HERO
    robotStates[Color::RED][1] = Robot(0, 0, 0, 2, 0, Color::RED);  // ENGINEER
    robotStates[Color::RED][2] = Robot(0, 0, 0, 3, 0, Color::RED);  // STANDARD
    robotStates[Color::RED][3] = Robot(0, 0, 0, 4, 0, Color::RED);  // STANDARD
    robotStates[Color::RED][4] = Robot(0, 0, 0, 5, 0, Color::RED);  // STANDARD
    robotStates[Color::RED][5] = Robot(0, 0, 0, 6, 0, Color::RED);  // Aireal
    robotStates[Color::RED][6] = Robot(0, 0, 0, 7, 0, Color::RED);  // sentry
    robotStates[Color::RED][7] = Robot(0, 0, 0, 8, 0, Color::RED);  //  BASE
    robotStates[Color::RED][8] = Robot(0, 0, 0, 9, 0, Color::RED);  // OUTPOST
    robotStates[Color::BLUE][0] = Robot(0, 0, 0, 1, 0, Color::BLUE);
    robotStates[Color::BLUE][1] = Robot(0, 0, 0, 2, 0, Color::BLUE);
    robotStates[Color::BLUE][2] = Robot(0, 0, 0, 3, 0, Color::BLUE);
    robotStates[Color::BLUE][3] = Robot(0, 0, 0, 4, 0, Color::BLUE);
    robotStates[Color::BLUE][4] = Robot(0, 0, 0, 5, 0, Color::BLUE);
    robotStates[Color::BLUE][5] = Robot(0, 0, 0, 6, 0, Color::BLUE);
    robotStates[Color::BLUE][6] = Robot(0, 0, 0, 7, 0, Color::BLUE);
    robotStates[Color::BLUE][7] = Robot(0, 0, 0, 8, 0, Color::BLUE);
    robotStates[Color::BLUE][8] = Robot(0, 0, 0, 9, 0, Color::BLUE);
}

RobotStates::~RobotStates() {
}

void RobotStates::setIndivualRobotState(Robot robot) { robotStates[robot.getColor()][robot.getNumber() - 1] = robot; }

Robot RobotStates::getIndivualRobotState(int robotNumber, Color color) { return robotStates[color][robotNumber - 1]; }

void RobotStates::updateRobotState(int robotNumber, Color color, int x, int y, int z, int health) {
    updateRobotStatePositon(robotNumber, color, x, y, z); 
    updateRobotStateHealth(robotNumber, color, health);
}

void RobotStates::updateRobotStatePositon(int number, Color color, int x, int y, int z) {
    robotStates[color][number-1].setX(x); 
    robotStates[color][number-1].setY(y); 
    robotStates[color][number-1].setZ(z); 
}

void RobotStates::updateRobotStateHealth(int number, Color color, int health) {
    robotStates[color][number - 1].setHealth(health); 
}

}  // namespace src::robotStates