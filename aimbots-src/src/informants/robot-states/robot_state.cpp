#include "robot_state.hpp"
namespace src::robotStates {
Robot::Robot(float x, float y, float z, int robotId, int health, Team teamColor) {
    this->position = Matrix<float, 3, 1>().zeroMatrix();
    position[Position::x][0] = x;
    position[Position::y][0] = y;
    position[Position::z][0] = z;
    this->robotID = robotID;
    this->health = health;

    this->teamColor = teamColor;
}

Robot::Robot(Matrix<float, 3, 1> position, int robotId, int health, Team teamColor) {
    this->position = position;
    this->robotID = robotID;
    this->health = health;

    this->teamColor = teamColor;
}

Robot::Robot() {
    position = Matrix<float, 3, 1>().zeroMatrix();
    robotID = -1;
    health = -1;
}

Robot::~Robot() {}

void Robot::setX(float x) { this->position[Position::x][0] = x; }

void Robot::setY(float y) { this->position[Position::y][0] = y; }

void Robot::setZ(float z) { this->position[Position::z][0] = z; }

void Robot::setPosition(float x, float y, float z) {
    position[Position::x][0] = x;
    position[Position::y][0] = y;
    position[Position::z][0] = z;
}

void Robot::setPosition(Matrix<float, 3, 1> position) { this->position = position; }

void Robot::setHealth(int health) { this->health = health; }

float Robot::getX() { return this->position[Position::x][0]; }

float Robot::getY() { return this->position[Position::y][0]; }

float Robot::getZ() { return this->position[Position::z][0]; }

Matrix<float, 3, 1> Robot::getPosition() { return this->position; }

int Robot::getHealth() { return this->health; }

int Robot::getRobotID() { return this->robotID; }

void Robot::setTeamColor(Team teamColor) { this->teamColor = teamColor; }

Team Robot::getTeamColor() { return this->teamColor; }

}  // namespace src::robotStates
