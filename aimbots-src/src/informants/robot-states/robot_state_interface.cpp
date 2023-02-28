#include "drivers.hpp"
// #include "ref_serial.hpp"
#include "informants/communication/communication_message.hpp"

#include "robot_state.hpp"
#include "robot_state_interface.hpp"
using namespace src::Communication;

namespace src::RobotStates {

RobotStates::RobotStates(src::Drivers* drivers) : drivers(drivers) {
    robotStates = Matrix<Robot, 2, 9>().zeroMatrix();
    robotStates[Team::RED][0] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 1, 0, Team::RED);  // HERO
    robotStates[Team::RED][1] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 2, 0, Team::RED);  // ENGINEER
    robotStates[Team::RED][2] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 3, 0, Team::RED);  // STANDARD
    robotStates[Team::RED][3] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 4, 0, Team::RED);  // STANDARD
    robotStates[Team::RED][4] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 5, 0, Team::RED);  // STANDARD
    robotStates[Team::RED][5] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 6, 0, Team::RED);  // Aerial
    robotStates[Team::RED][6] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 7, 0, Team::RED);  // sentry
    robotStates[Team::RED][7] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 8, 0, Team::RED);  //  BASE
    robotStates[Team::RED][8] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 9, 0, Team::RED);  // OUTPOST
    robotStates[Team::BLUE][0] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 1, 0, Team::BLUE);
    robotStates[Team::BLUE][1] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 2, 0, Team::BLUE);
    robotStates[Team::BLUE][2] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 3, 0, Team::BLUE);
    robotStates[Team::BLUE][3] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 4, 0, Team::BLUE);
    robotStates[Team::BLUE][4] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 5, 0, Team::BLUE);
    robotStates[Team::BLUE][5] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 6, 0, Team::BLUE);
    robotStates[Team::BLUE][6] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 7, 0, Team::BLUE);
    robotStates[Team::BLUE][7] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 8, 0, Team::BLUE);
    robotStates[Team::BLUE][8] = Robot(Matrix<short, 3, 1>().zeroMatrix(), 9, 0, Team::BLUE);
}

RobotStates::~RobotStates() {}

void RobotStates::setIndivualRobotState(Robot robot) { robotStates[robot.getTeamColor()][robot.getRobotID() - 1] = robot; }

Robot RobotStates::getIndivualRobotState(int robotNumber, Team team) { return robotStates[team][robotNumber - 1]; }

void RobotStates::updateRobotState(int robotNumber, Team teamColor, short x, short y, short z, int health) {
    updateRobotStatePosition(robotNumber, teamColor, x, y, z);
    updateRobotStateHealth(robotNumber, teamColor, health);
}

void RobotStates::updateRobotStatePosition(int number, Team teamColor, short x, short y, short z) {
    robotStates[teamColor][number - 1].setX(x);
    robotStates[teamColor][number - 1].setY(y);
    robotStates[teamColor][number - 1].setZ(z);
}

void RobotStates::updateRobotStateHealth(int number, Team teamColor, int health) { robotStates[teamColor][number - 1].setHealth(health); }

#ifdef TARGET_SENTRY
robot_state_message_team RobotStates::createMessage() {
    // will update all of the struct messages for the coms
    tap::communication::serial::RefSerial::RobotId id = drivers->refSerial.getRobotData().robotId;
    Team color = tap::communication::serial::RefSerial::isBlueTeam(id) ? Team::BLUE : Team::RED;

    robot_state_message_team message;

    message.standardX = robotStates[color][2].getX();
    message.standardY = robotStates[color][2].getY();
    message.heroX = robotStates[color][0].getX();
    message.heroY = robotStates[color][0].getY();
    message.sentryX = robotStates[color][6].getX();
    message.sentryY = robotStates[color][6].getY();

    return message;
}
#else

#endif

void updateRobotStateHero() {}
void updateRobotStateStandard() {}
void updateRobotStateSentry() {}

}  // namespace src::RobotStates