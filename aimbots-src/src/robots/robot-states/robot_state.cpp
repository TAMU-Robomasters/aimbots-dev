#include "robot_state.hpp"
namespace src::robotStates {
Robot::Robot(int x, int y, int z, int number, int health, Color color) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->number = number;
    this->health = health;

    this->color = color;
}

Robot::Robot() {
    x = 0;
    y = 0;
    z = 0;
    number = -1;
}

Robot::~Robot() {}

void Robot::setX(int x) { this->x = x; }

void Robot::setY(int y) { this->y = y; }

void Robot::setZ(int z) { this->z = z; }

void Robot::setHealth(int health) { this->health = health; }

int Robot::getX() { return this->x; }

int Robot::getY() { return this->y; }

int Robot::getZ() { return this->z; }

int Robot::getHealth() { return this->health; }

int Robot::getNumber() { return this->number; }

void Robot::setColor(Color color) { this->color = color; }

Color Robot::getColor() { return this->color; }

}  // namespace src::robotStates
