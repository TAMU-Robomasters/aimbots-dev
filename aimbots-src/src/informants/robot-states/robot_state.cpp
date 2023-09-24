#ifdef REF_COMM_COMPATIBLE

#include "robot_state.hpp"

namespace src::RobotStates {
Robot::Robot(short x, short y, short z, int robotId, int health, Team teamColor) {
    this->position = Matrix<short, 3, 1>().zeroMatrix();
    position[Position::x][0] = x;
    position[Position::y][0] = y;
    position[Position::z][0] = z;
    this->robotID = robotID;
    this->health = health;

    this->teamColor = teamColor;
}

Robot::Robot(Matrix<short, 3, 1> position, int robotId, int health, Team teamColor) {
    this->position = position;
    this->robotID = robotID;
    this->health = health;

    this->teamColor = teamColor;
}

Robot::Robot() {
    position = Matrix<short, 3, 1>().zeroMatrix();
    robotID = -1;
    health = -1;
}

Robot::~Robot() {}

void Robot::setX(short x) { this->position[Position::x][0] = x; }

void Robot::setY(short y) { this->position[Position::y][0] = y; }

void Robot::setZ(short z) { this->position[Position::z][0] = z; }

void Robot::setPosition(short x, short y, short z) {
    position[Position::x][0] = x;
    position[Position::y][0] = y;
    position[Position::z][0] = z;
}

void Robot::setPosition(Matrix<short, 3, 1> position) { this->position = position; }

void Robot::setHealth(int health) { this->health = health; }

short Robot::getX() { return this->position[Position::x][0]; }

short Robot::getY() { return this->position[Position::y][0]; }

short Robot::getZ() { return this->position[Position::z][0]; }

Matrix<short, 3, 1> Robot::getPosition() { return this->position; }

int Robot::getHealth() { return this->health; }

int Robot::getRobotID() { return this->robotID; }

void Robot::setTeamColor(Team teamColor) { this->teamColor = teamColor; }

Team Robot::getTeamColor() { return this->teamColor; }

}  // namespace src::RobotStates

#endif //#ifdef REF_COMM_COMPATIBLE
