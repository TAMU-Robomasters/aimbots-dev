#pragma once

#include "utils/tools/common_types.hpp"

#include "drivers.hpp"


namespace src::Utils::Control::PowerLimiting {

/* 
 * Documentation & comments last updated 20 Nov 2024
 * - Seth Mayhue
 * - Christopher Muniz
 * - Dimitri Diniaco
 * 
 * //// What is power limiter? ////
 * 
 * Power limiter is a game mechanic that focuses on how much energy 
 *   a robot can burn through until experiencing HP reduction
 * - More notes taken in the Google Drive (TAMU Robomasters\RM - 2024-2025\Software\Embedded\'Power Limiting Notes')
 * - Power limit info located in sec 5.1.3, 2024 Official English Championship Rules Manual v1.0.0
 * 
 * What this file does:
 * - Locally calulate the energy buffer
 * - Return a coefficient to be applied to the amt of energy provided to the motors
 *   - If we are close to burning thru our buffer, we will cut down on energy output
 *   - Else if we are above our buffer due to a bonus (i.e. ramp bonus (2024)), we will increase energy output
 *     - Ramp bonus info located in sec 5.6.3.5, 2024 Official English Championship Rules Manual v1.0.0
 *   - Else we continue with regular output
 * 
 * Important features:
 * - Theoretically accounts for supercaps output, as calculations for energy buffer are done with output from batter
 *   - To be tested in Spring 2024
 * - Theoretically Works with mecanum + swerve drivetrains
 *   - Swerve yaw motors do not get scaled, only drive motors get scaled
 *   - To be tested in Spring 2024
 */

class PowerLimiter {
public:
    // Constructor
    PowerLimiter(
        const src::Drivers* drivers,
        float startingEnergyBuffer,
        float energyBufferLimitThreshold,
        float energyBufferCritThreshold,
        float safetyFactor,
        float excessBufferConsumption);

    // Returns ratio of what our power output should be
    //   If we're getting close to breaking the energy buffer, it cuts down on power
    //   Else if we're above our buffer (>60J), it cuts into power
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
    float energyBufferLimitThreshold; // Average max energy buffer for a robot?
                                      //   Not entirely sure. 60J across robots iirc ._.
    float energyBufferCritThreshold;  // Critical buffer trigger
                                      //   If we are below this threshold, we will limit power
                                      //   Kind of like how 25% HP in Lethal Company says "critical" on your screen

    // Ramp bonus variables
    bool hasRampBonus;               // Do we actively have a ramp bonus? (T/F)
    uint32_t rampBonusStartTime;     // Starting time of ramp bonus
    uint32_t rampBonusTimeDuration;  // Time remaining of ramp bonus. 
                                     //   Initialized in updatePowerAndEnergyBuffer.
    float excessBufferConsumption;   // Ratio for how fast we should deplete our energy buffer bonus
                                     //   Will default to '2.0' for applicable robots (standard, hero, sentry)
                                     //   May be reclassified here if more energy buffer bonuses get added into the game

    // Recalculates consumedPower & updates energyBuffer
    //   If ref server has new energy buffer, we use that
    //   Otherwise, recalculate locally
    void updatePowerAndEnergyBuffer();
};

};  // namespace src::Utils::Control::PowerLimiting