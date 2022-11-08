#pragma once

enum Color {
    RED = 0,
    BLUE =1,
    UNKNOWN = 2
};

class robot {
private:
    int number;
    int x, y, z;
    int health;

    Color color;

public:
    robot(int x, int y, int z, int number, int health, Color color);
    robot();
    ~robot();

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
