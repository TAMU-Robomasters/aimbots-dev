#include "drivers.hpp"

#include "robot_state.hpp"
#include "robot_states_interface.hpp"

namespace src::robotStates {

RobotStates::RobotStates(src::Drivers* drivers) : drivers(drivers) {
    robotStates = Matrix<Robot, 2, 9>().zeroMatrix();
    robotStates[Team::RED][0] = Robot(Matrix<float,3,1>().zeroMatrix(), 1, 0, Team::RED);  // HERO
    robotStates[Team::RED][1] = Robot(Matrix<float,3,1>().zeroMatrix(), 2, 0, Team::RED);  // ENGINEER
    robotStates[Team::RED][2] = Robot(Matrix<float,3,1>().zeroMatrix(), 3, 0, Team::RED);  // STANDARD
    robotStates[Team::RED][3] = Robot(Matrix<float,3,1>().zeroMatrix(), 4, 0, Team::RED);  // STANDARD
    robotStates[Team::RED][4] = Robot(Matrix<float,3,1>().zeroMatrix(), 5, 0, Team::RED);  // STANDARD
    robotStates[Team::RED][5] = Robot(Matrix<float,3,1>().zeroMatrix(), 6, 0, Team::RED);  // Aerial
    robotStates[Team::RED][6] = Robot(Matrix<float,3,1>().zeroMatrix(), 7, 0, Team::RED);  // sentry
    robotStates[Team::RED][7] = Robot(Matrix<float,3,1>().zeroMatrix(), 8, 0, Team::RED);  //  BASE
    robotStates[Team::RED][8] = Robot(Matrix<float,3,1>().zeroMatrix(), 9, 0, Team::RED);  // OUTPOST
    robotStates[Team::BLUE][0] = Robot(Matrix<float,3,1>().zeroMatrix(), 1, 0, Team::BLUE);
    robotStates[Team::BLUE][1] = Robot(Matrix<float,3,1>().zeroMatrix(), 2, 0, Team::BLUE);
    robotStates[Team::BLUE][2] = Robot(Matrix<float,3,1>().zeroMatrix(), 3, 0, Team::BLUE);
    robotStates[Team::BLUE][3] = Robot(Matrix<float,3,1>().zeroMatrix(), 4, 0, Team::BLUE);
    robotStates[Team::BLUE][4] = Robot(Matrix<float,3,1>().zeroMatrix(), 5, 0, Team::BLUE);
    robotStates[Team::BLUE][5] = Robot(Matrix<float,3,1>().zeroMatrix(), 6, 0, Team::BLUE);
    robotStates[Team::BLUE][6] = Robot(Matrix<float,3,1>().zeroMatrix(), 7, 0, Team::BLUE);
    robotStates[Team::BLUE][7] = Robot(Matrix<float,3,1>().zeroMatrix(), 8, 0, Team::BLUE);
    robotStates[Team::BLUE][8] = Robot(Matrix<float,3,1>().zeroMatrix(), 9, 0, Team::BLUE);
}

RobotStates::~RobotStates() {}

void RobotStates::setIndivualRobotState(Robot robot) { robotStates[robot.getTeamColor()][robot.getRobotID() - 1] = robot; }

Robot RobotStates::getIndivualRobotState(int robotNumber, Team team) { return robotStates[team][robotNumber - 1]; }

void RobotStates::updateRobotState(int robotNumber, Team teamColor, float x, float y, float z, int health) {
    updateRobotStatePosition(robotNumber, teamColor, x, y, z);
    updateRobotStateHealth(robotNumber, teamColor, health);
}

void RobotStates::updateRobotStatePosition(int number, Team teamColor, float x, float y, float z) {
    robotStates[teamColor][number - 1].setX(x);
    robotStates[teamColor][number - 1].setY(y);
    robotStates[teamColor][number - 1].setZ(z);
}

void RobotStates::updateRobotStateHealth(int number, Team teamColor, int health) { robotStates[teamColor][number - 1].setHealth(health); }

void updateRefSystem() {
    // if (driver->refSerial.getSerialReceivingData()) {
    //     driver->refSerial.getRobotDate().RobotData.
    // }
}

}  // namespace src::robotStates