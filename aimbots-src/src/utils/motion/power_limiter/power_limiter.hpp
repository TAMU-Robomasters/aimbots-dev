#pragma once

#include "utils/tools/common_types.hpp"

#include "drivers.hpp"


namespace src::Utils::Control::PowerLimiting {

/* 
 * Documentation & comments last updated 9 Nov 2024
 * - Seth Mayhue
 * - Christopher Muniz
 * - Dimitri Diniaco
 */

class PowerLimiter {
public:
    // Constructor
    PowerLimiter(
        const src::Drivers* drivers,
        float startingEnergyBuffer,
        float energyBufferLimitThreshold,
        float energyBufferCritThreshold,
        float safetyFactor);

    // Returns ratio of what our power output should be
    //   If we're getting close to breaking the energy buffer, it cuts down on power
    //   Otherwise, robot keeps going
    //
    // Relative to the codebase, this function is only used in ChassisSubsystem::limitChassisPower().
    //   It is used to cut the power given to the chassis wheels of a robot
    //   if and only if a power cut needs to be made
    float getPowerLimitRatio();

private:
    const src::Drivers* drivers; // Drivers data (WOW CRAZY)
    
    float energyBuffer; // Energy buffer mechanic
                        //   Initialized with startingEnergyBuffer
                        //   Updated with robotData.chassisData.powerBuffer upon ref system update
                        //   Locally calculated between ref system updates
    
    // Local calculation of energy buffer variables
    const float startingEnergyBuffer;        // Initialized energy buffer at the start of a match (60J)
    float consumedPower;                     // Power consumed between each local refresh.
    float safetyFactor;                      // Used for minimizing interpolation error in calculating energy buffer.
    uint32_t prevTime;                       // Used for calculating power used in a given time frame (power = current/time)
    uint32_t prevRobotDataReceivedTimestamp; // Most recent time stamp of ref data. If new data is in, we get energy buffer from ref system

    // TODO: verify these 2 variables definition
    // Power limiting ratio calculation variables
    //   Will need to verify/update later
    float energyBufferLimitThreshold; // ???
    float energyBufferCritThreshold;  // Critical buffer trigger
                                      //   If we are below this threshold, we will limit power
                                      //   Kind of like how 25% HP in Lethal Company says "critical" on your screen

    // Ramp bonus variables
    bool hasRampBonus;                       // Do we actively have a ramp bonus? (T/F)
    uint32_t rampBonusStartTime;             // Starting time of ramp bonus
    uint32_t rampBonusTimeDuration;          // Time remaining of ramp bonus. 
                                             //   Initialized in updatePowerAndEnergyBuffer.

    // Recalculates consumedPower & updates energyBuffer
    //   If ref server has new energy buffer, we use that
    //   Otherwise, recalculate locally
    void updatePowerAndEnergyBuffer();
};

};  // namespace src::Utils::Control::PowerLimiting