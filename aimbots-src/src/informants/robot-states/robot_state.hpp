#pragma once
#include "utils/common_types.hpp"

namespace src::RobotStates {
enum Team { RED = 0, BLUE = 1 };
enum Position { x = 0, y = 1, z = 2 };

class Robot {
private:
    int robotID;
    Matrix<short, 3, 1> position;
    int health;

    Team teamColor;

public:
    Robot(short x, short y, short z, int robotID, int health, Team teamColor);
    Robot(Matrix<short, 3, 1> position, int robotID, int health, Team teamColor);
    Robot();
    ~Robot();

    void setX(short x);
    void setY(short y);
    void setZ(short z);
    void setPosition(short x, short y, short z);
    void setPosition(Matrix<short, 3, 1> position);
    void setHealth(int health);
    void setTeamColor(Team teamColor);

    short getX();
    short getY();
    short getZ();
    Matrix<short, 3, 1> getPosition();
    int getHealth();
    int getRobotID();
    Team getTeamColor();
};
}  // namespace src::robotStates