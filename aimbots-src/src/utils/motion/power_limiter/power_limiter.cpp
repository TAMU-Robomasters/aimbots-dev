#include "power_limiter.hpp"

namespace src::Utils::Control::PowerLimiting {
PowerLimiter::PowerLimiter(
    const src::Drivers *drivers,
    float startingEnergyBuffer,
    float energyBufferLimitThreshold,
    float energyBufferCritThreshold,
    float safetyFactor)
    : drivers(drivers),
      startingEnergyBuffer(startingEnergyBuffer),
      energyBufferLimitThreshold(energyBufferLimitThreshold),
      energyBufferCritThreshold(energyBufferCritThreshold),
      consumedPower(0.0f),
      safetyFactor(safetyFactor),
      prevTime(0),
      prevRobotDataReceivedTimestamp(0)
//
{}

float PowerLimiter::getPowerLimitRatio() {
    // Catch for finding new ref info
    if (!drivers->refSerial.getRefSerialReceivingData()) {
        return 1.0f;
    }

    // Updates power expended since previous local refresh & energy buffer
    updatePowerAndEnergyBuffer();

    // Checks if we are cutting close to breaking the 60/250J limit
    //    If we're close, it cuts down on power
    //    If we're not, robot keeps truckin' along
    if (energyBuffer < energyBufferLimitThreshold) {
        return limitVal(
            static_cast<float>(energyBuffer - energyBufferCritThreshold) / energyBufferLimitThreshold,
            0.0f,
            1.0f);
    } else {
        return 1.0f;
    }
}

void PowerLimiter::updatePowerAndEnergyBuffer() {

    // Calculates power consumed between local refresh
    const auto &robotData = drivers->refSerial.getRobotData();                 // Pulls current data
    const auto &chassisData = robotData.chassis;                               //
    const float current = chassisData.current;                                 //
    const float newChassisPower = (chassisData.volt * current) / 1'000'000.0f; // *1'000'000.0f = convert microwatts to watts
    
    // Calculates energy buffer in between ref info updates
    const float dt = tap::arch::clock::getTimeMilliseconds() - prevTime;
    prevTime = tap::arch::clock::getTimeMilliseconds();
    energyBuffer -= ((consumedPower * safetyFactor) - chassisData.powerConsumptionLimit) * dt / 1000.0f;

    // Checks for ref info update. If ref info update is available, it resets our energy buffer
    if (robotData.robotDataReceivedTimestamp != prevRobotDataReceivedTimestamp) {
        energyBuffer = chassisData.powerBuffer;
        prevRobotDataReceivedTimestamp = robotData.robotDataReceivedTimestamp;
    }

    // Updates consumed power
    consumedPower = newChassisPower;
}

}  // namespace src::Utils::Control::PowerLimiting