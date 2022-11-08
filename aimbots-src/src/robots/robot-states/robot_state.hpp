#pragma once
namespace src::robotStates {
enum Color { RED = 0, BLUE = 1, UNKNOWN = 2 };

class Robot {
private:
    int number;
    int x, y, z;
    int health;

    Color color;

public:
    Robot(int x, int y, int z, int number, int health, Color color);
    Robot();
    ~Robot();

    void setX(int x);
    void setY(int y);
    void setZ(int z);
    void setHealth(int health);
    void setColor(Color color);

    int getX();
    int getY();
    int getZ();
    int getHealth();
    int getNumber();
    Color getColor();
};
}  // namespace src::RobotStates