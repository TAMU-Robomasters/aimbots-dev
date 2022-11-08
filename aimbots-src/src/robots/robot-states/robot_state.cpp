#include "robot_state.hpp"

robot::robot(int x, int y, int z, int number, int health, Color color) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->number = number;
    this->health = health;

    this->color = color;
}

robot::robot() {
    x = 0;
    y = 0;
    z = 0;
    number = -1;
}

robot::~robot() {}

void robot::setX(int x) { this->x = x; }

void robot::setY(int y) { this->y = y; }

void robot::setZ(int z) { this->z = z; }

void robot::setHealth(int health) { this->health = health; }

int robot::getX() { return this->x; }

int robot::getY() { return this->y; }

int robot::getZ() { return this->z; }

int robot::getHealth() { return this->health; }

int robot::getNumber() { return this->number; }

void robot::setColor(Color color) { this->color = color; }

Color robot::getColor() { return this->color; }


