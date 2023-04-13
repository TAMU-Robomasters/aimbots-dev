#pragma once

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

#include "tap/communication/serial/ref_serial.hpp"

#include "drivers.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

enum barrelPosition {
    LEFT = 0,
    RIGHT = 1,
    CURRENT = -1,
};

namespace src::Barrel_Manager {

class BarrelManagerSubsystem : public tap::control::Subsystem {
public:
    BarrelManagerSubsystem(tap::Drivers* drivers);

    mockable void initialize() override;
    void refresh() override;

    const char* getName() override { return "Barrel Manager Subsystem"; }

    inline bool isOnline() const { return swapMotor.isMotorOnline(); }

    void setMotorOutput(float output);

    float getMotorOutput();

    //Returns encoder value of motor
    float getMotorPosition();

    //Runs into hard stops on both sides of lead screw to find their position
    bool findZeroPosition();

    //Finds which barrel is equipped
    barrelPosition getPosition();

    //Chooses to equip a specific barrel
    void setPosition(barrelPosition pos);

    //Will toggle which barrel is equipped
    void togglePosition();

    //If no position specified, defaults to -1, which means get currently equipped barrel
    float getBarrelHeat(barrelPosition pos);

    //Returns true when barrel is aligned with the flywheels
    bool isBarrelAligned();

private:
    tap::Drivers* drivers;
    barrelPosition currentBarrelPosition = LEFT;

    DJIMotor swapMotor;

    int barrelState = 2;
    //2 = Standard operation
    //0 = finding Left Barrel Stop
    //1 = finding Right Barrel Stop

    float targetSwapMotorPosition; //In mm

    float currentSwapMotorPosition; //In mm

    float desiredSwapMotorOutput;

    float limitLRPositions[2] = {0,1000}; // {Left Pos, Right Pos} In mm, should be determined in the code at launch

};

}  // namespace src::Shooter
#endif