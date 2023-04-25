#pragma once

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

#include "tap/communication/serial/ref_serial.hpp"

#include "drivers.hpp"

#ifdef BARREL_SWAP_COMPATIBLE



namespace src::Barrel_Manager {

enum barrelSide {
    LEFT = 0,
    RIGHT = 1,
    CURRENT = -1,
};

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
    bool findZeroPosition(barrelSide stopSideToFind);

    //Finds which barrel is equipped
    barrelSide getSide();

    //Chooses to equip a specific barrel
    void setSide(barrelSide side);

    //Will toggle which barrel is equipped
    void toggleSide();

    //If no position specified, defaults to -1, which means get currently equipped barrel
    float getBarrelHeat(barrelSide side);

    //Returns true when barrel is aligned with the flywheels
    bool isBarrelAligned();

    //TODO: Make sure Left and Right offsets are added/subtracted correctly
    float getSideInMM(barrelSide side) {return (side == barrelSide::LEFT) ? (limitLRPositions[barrelSide::LEFT] + LEFT_STOP_OFFSET) : (limitLRPositions[barrelSide::RIGHT] - RIGHT_STOP_OFFSET);}

    

private:
    tap::Drivers* drivers;
    barrelSide currentBarrelSide = barrelSide::LEFT;

    DJIMotor swapMotor;

    int barrelState = 2;
    //0 = finding Left Barrel Stop
    //1 = finding Right Barrel Stop
    //2 = Standard operation

    float targetSwapMotorPosition; //In mm

    float currentSwapMotorPosition; //In mm

    float desiredSwapMotorOutput;

    float limitLRPositions[2] = {0,1000000000}; // {Left side, Right side} In mm, TODO: should be determined in the code at launch



};

}  // namespace src::Shooter
#endif