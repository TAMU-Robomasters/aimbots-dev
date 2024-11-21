#include "power_limiter.hpp"

namespace src::Utils::Control::PowerLimiting {
PowerLimiter::PowerLimiter(
    const src::Drivers *drivers,
    float startingEnergyBuffer,
    float energyBufferLimitThreshold,  
    float energyBufferCritThreshold,   
    float safetyFactor)               
    : drivers(drivers),
      energyBuffer( startingEnergyBuffer ), 
      startingEnergyBuffer( startingEnergyBuffer ),
      energyBufferLimitThreshold(energyBufferLimitThreshold),
      energyBufferCritThreshold(energyBufferCritThreshold),
      consumedPower(0.0f),
      safetyFactor(safetyFactor),
      prevTime(0),
      prevRobotDataReceivedTimestamp(0),
      hasRampBonus(false)
//
{}

float PowerLimiter::getPowerLimitRatio() {
    // Catch for not having connection to ref system
    if (!drivers->refSerial.getRefSerialReceivingData()) {
        return 1.0f;
    }

    // Updates power expended since previous local refresh & energy buffer
    updatePowerAndEnergyBuffer();

    // TODO: confirm & update documentation below for returns
    // Checks if we are cutting close to burning thru our energy buffer ()
    //    If we're close, it cuts down on power
    //    If we're not, robot keeps truckin' along
    if (energyBuffer < energyBufferLimitThreshold) {
        return limitVal(
            static_cast<float>(energyBuffer - energyBufferCritThreshold) / energyBufferLimitThreshold,
            0.0f,
            1.0f);
    } 
    else {
        return 1.0f;
    }
}

void PowerLimiter::updatePowerAndEnergyBuffer() {

    // Calculates power consumed between local refreshes
    const auto &robotData = drivers->refSerial.getRobotData();                             // Pulls electrical current data from chassis
    const auto &chassisData = robotData.chassis;                                           //    & calculates power consumed since last time stamp
    const float newChassisPower = (chassisData.volt * chassisData.current) / 1'000'000.0f; // <- Div by 1'000'000.0f = convert microwatts to watts
    
    // Calculates energy buffer between ref info updates
    const float dt = tap::arch::clock::getTimeMilliseconds() - prevTime;                                 // Calculates time passed between local cycles
    prevTime = tap::arch::clock::getTimeMilliseconds();                                                  // 
    energyBuffer -= ((consumedPower * safetyFactor) - chassisData.powerConsumptionLimit) * dt / 1000.0f; // Recalculates energy buffer

    // Accounts for ramp bonus
    if(energyBuffer > startingEnergyBuffer && !hasRampBonus)          // Checks if we have > 60J && we don't have a bonus
    {
        rampBonusStartTime = tap::arch::clock::getTimeMilliseconds(); // Starts ramp bonus clock
        hasRampBonus = true;                                          // Local indicator stating robot have ramp bonus
    }
    rampBonusTimeDuration = 20'000 
            - (tap::arch::clock::getTimeMilliseconds() - rampBonusStartTime); // Calculate time remaining of ramp bonus
    if( hasRampBonus && (rampBonusTimeDuration < 0 || energyBuffer < 60)  )   // Checks if our duration has expired or we have burned thru our bonus
    {
        if(energyBuffer > startingEnergyBuffer)      // If we have more than 60J buffer
            energyBuffer = startingEnergyBuffer;     //   -> set to 60J
        hasRampBonus = false;                        // Local indicator stating robot does not have ramp bonus
    }


    // Checks for ref info update.
    //   If ref info update is available, it updates our energy buffer
    if (robotData.robotDataReceivedTimestamp != prevRobotDataReceivedTimestamp) {
        energyBuffer = chassisData.powerBuffer;
        prevRobotDataReceivedTimestamp = robotData.robotDataReceivedTimestamp;
    }

    // Updates consumed power
    consumedPower = newChassisPower;
}

}  // namespace src::Utils::Control::PowerLimiting 