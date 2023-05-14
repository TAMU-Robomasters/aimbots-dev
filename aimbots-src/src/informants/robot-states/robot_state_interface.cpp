#include "drivers.hpp"
// #include "ref_serial.hpp"

#include "robot_state.hpp"
#include "robot_state_interface.hpp"
using namespace src::Communication;

namespace src::RobotStates {

RobotStates::RobotStates(tap::Drivers& drivers) : tap::control::Subsystem(&drivers), drivers(drivers), messageHandler(drivers) {
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
void RobotStates::updateTeamMessage() {
    // will update all of the struct messages for the coms
    // tap::communication::serial::RefSerial::RobotId id = &drivers->refSerial.getRobotData().robotId;
    Team color = /*tap::communication::serial::RefSerial::isBlueTeam(id) ? Team::BLUE : */ Team::RED;
    teamMessage[0] = 0;
    teamMessage[1] = robotStates[color][2].getX() >> 8;
    teamMessage[2] = robotStates[color][2].getX();
    teamMessage[3] = robotStates[color][2].getY() >> 8;
    teamMessage[4] = robotStates[color][2].getY();
    teamMessage[5] = robotStates[color][0].getX() >> 8;
    teamMessage[6] = robotStates[color][0].getX();
    teamMessage[7] = robotStates[color][0].getY() >> 8;
    teamMessage[8] = robotStates[color][0].getY();
    teamMessage[9] = robotStates[color][6].getX() >> 8;
    teamMessage[10] = robotStates[color][6].getX();
    teamMessage[11] = robotStates[color][6].getY() >> 8;
    teamMessage[12] = robotStates[color][6].getY();
    // message.standardX = robotStates[color][2].getX();
    // message.standardY = robotStates[color][2].getY();
    // message.heroX = robotStates[color][0].getX();
    // message.heroY = robotStates[color][0].getY();
    // message.sentryX = robotStates[color][6].getX();
    // message.sentryY = robotStates[color][6].getY();
}
#elif TARGET_STANDARD
void updateStandardMessage() {
    Team color = /*tap::communication::serial::RefSerial::isBlueTeam(id) ? Team::BLUE : */ Team::RED;
    // standardMessage[0] = 1;
    // standardMessage[1] = robotStates[color][2].getX() >> 8;
    // standardMessage[2] = robotStates[color][2].getX();
    // standardMessage[3] = robotStates[color][2].getY() >> 8;
    // standardMessage[4] = robotStates[color][2].getY();
}

#elif TRAGET_HERO

void updateHeroMessage() {
    Team color = /*tap::communication::serial::RefSerial::isBlueTeam(id) ? Team::BLUE : */ Team::RED;
    // heroMessage[0] = 1;
    // heroMessage[1] = robotStates[color][2].getX() >> 8;
    // heroMessage[2] = robotStates[color][2].getX();
    // heroMessage[3] = robotStates[color][2].getY() >> 8;
    // heroMessage[4] = robotStates[color][2].getY();
}
#endif

void RobotStates::respond() {
    // have it call the handler which will return the a struct for robot positions
    // messageHandler;
    // update the values in the robot states
}

// void updateRobotStateHero() {}
// void updateRobotStateStandard() {}
// void updateRobotStateSentry() {}

void RobotStates::refresh() {
    this->respond();
#ifdef TARGET_SENTRY
    // TODO:: the code that will update stuff based on the recived messages
    this->updateTeamMessage();
#elif TARGET_STANDARD
    this->updateStandardMessage();
#elif TRAGET_HERO
    this->updateHeroMessage();
#endif
}
}  // namespace src::RobotStates