#pragma once

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "tap/communication/serial/ref_serial.hpp"

#include "drivers.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace src::BarrelManager {

//To the Left is negative encoder counts
//To the Right is positive encoder counts
enum barrelSide {
    LEFT = 0,
    RIGHT = 1,
    CURRENT = -1,
};

class BarrelManagerSubsystem : public tap::control::Subsystem {
public:
    BarrelManagerSubsystem(tap::Drivers* drivers,
    float HARD_STOP_OFFSET,
    float BARREL_SWAP_DISTANCE_MM,
    float BARRELS_ALIGNED_TOLERANCE,
    float LEAD_SCREW_TICKS_PER_MM,
    int16_t LEAD_SCREW_CURRENT_SPIKE_TORQUE,
    int16_t LEAD_SCREW_CALI_OUTPUT,
    SmoothPIDConfig BARREL_SWAP_POSITION_PID_CONFIG,
    std::array<BarrelID, 2> BARREL_ARRAY,
    BarrelID &currentBarrel);

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

    //Returns true when barrel is aligned with the flywheels (with a tolerance)
    bool isBarrelAligned();

    //Give a barrel side, and in return get the mm position that side corresponds to
    float getSideInMM(barrelSide side) {return (side == barrelSide::LEFT) ? (limitLRPositions[barrelSide::LEFT]) : (limitLRPositions[barrelSide::RIGHT]);}

    //Returns the ID of the barrel corresponding to the side
    BarrelID getBarrelSideID(barrelSide side = barrelSide::CURRENT) {
        barrelSide actualSide = side;
        if (actualSide == barrelSide::CURRENT) {
            actualSide = getSide();
        }
        return BARREL_ARRAY[actualSide];
    }

private:
    tap::Drivers* drivers;
    barrelSide currentBarrelSide = barrelSide::LEFT;

    MilliTimeout currentSpikeTimer;

    DJIMotor swapMotor;

    int barrelState = 2;
    //0 = finding Left Barrel Stop
    //1 = finding Right Barrel Stop
    //2 = Standard operation

    float targetSwapMotorPosition; //In mm

    float currentSwapMotorPosition; //In mm

    float desiredSwapMotorOutput;

    // {Left side, Right side} In mm
    float limitLRPositions[2] = {0,BARREL_SWAP_DISTANCE_MM}; // 

    //Constants to be set by at the construction of a new barrel manager instance
    float HARD_STOP_OFFSET;
    float BARREL_SWAP_DISTANCE_MM;
    float BARRELS_ALIGNED_TOLERANCE;
    float LEAD_SCREW_TICKS_PER_MM;
    int16_t LEAD_SCREW_CURRENT_SPIKE_TORQUE;
    int16_t LEAD_SCREW_CALI_OUTPUT;
    SmoothPIDConfig BARREL_SWAP_POSITION_PID_CONFIG;

    std::array<BarrelID, 2> BARREL_ARRAY;
    BarrelID &currentBarrel;
};

}  // namespace src::Shooter
#endif