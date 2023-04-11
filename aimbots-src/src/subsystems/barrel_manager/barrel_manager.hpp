#pragma once

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

enum location {
    LEFT = 0,
    RIGHT = 1,
    CURRENT = -1,
};

namespace src::Barrel_Manager {

class BarrelSwapSubsytem : public tap::control::Subsystem {
public:
    BarrelSwapSubsytem(tap::Drivers* drivers);

    mockable void initialize() override;
    void refresh() override;

    const char* getName() override { return "Barrel Manager Subsystem"; }

    inline bool isOnline() const { return swapMotor.isMotorOnline(); }

    //Applies power to motor
    void setMotorRPM(float rpm);

    float getMotorRPM();

    //Returns encoder value of motor
    float getMotorPosition();

    //Runs into hard stops on both sides of lead screw to find their position
    bool findZeroPosition();

    //Finds which barrel is equipped
    location getPosition();

    //Chooses to equip a specific barrel
    void setPosition(location pos);

    //Will toggle which barrel is equipped
    void togglePosition();

    //If no position specified, defaults to -1, which means get currently equipped barrel
    float getBarrelHeat(location pos = CURRENT);

private:
    tap::Drivers* drivers;
    location current_position;

    DJIMotor swapMotor;

    float targetSwapMotorPosition; //In mm

    float currentSwapMotorPosition; //In mm

    float desiredSwapMotorOutput;


};

}  // namespace src::Shooter
#endif