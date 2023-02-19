#pragma once
#include "utils/common_types.hpp"

namespace src::RobotStates {
enum Team { RED = 0, BLUE = 1 };
enum Position { x = 0, y = 1, z = 2 };

class Robot {
private:
    int robotID;
    Matrix<float, 3, 1> position;
    int health;

    Team teamColor;

public:
    Robot(float x, float y, float z, int robotID, int health, Team teamColor);
    Robot(Matrix<float, 3, 1> position, int robotID, int health, Team teamColor);
    Robot();
    ~Robot();

    void setX(float x);
    void setY(float y);
    void setZ(float z);
    void setPosition(float x, float y, float z);
    void setPosition(Matrix<float, 3, 1> position);
    void setHealth(int health);
    void setTeamColor(Team teamColor);

    float getX();
    float getY();
    float getZ();
    Matrix<float, 3, 1> getPosition();
    int getHealth();
    int getRobotID();
    Team getTeamColor();
};
}  // namespace src::robotStates