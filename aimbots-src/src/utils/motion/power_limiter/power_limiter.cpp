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
float energyBufferDisplay = 0.0f;
float limitRatioDisplay = 0.0f;


float PowerLimiter::getPowerLimitRatio() {
    if (!drivers->refSerial.getRefSerialReceivingData()) {
        return 1.0f;
    }

    updatePowerAndEnergyBuffer();
    limitRatioDisplay = static_cast<float>(energyBuffer - energyBufferCritThreshold) / energyBufferLimitThreshold;

    if (energyBuffer < energyBufferLimitThreshold) {
        return 1.0f;
        // return limitVal(
        //     static_cast<float>(energyBuffer - energyBufferCritThreshold) / energyBufferLimitThreshold,
        //     0.0f,
        //     1.0f);
    } else {
        return 1.0f;
    }
}


void PowerLimiter::updatePowerAndEnergyBuffer() {
    const auto &robotData = drivers->refSerial.getRobotData();
    const auto &chassisData = robotData.chassis;
    const float current = chassisData.current;
    const float newChassisPower = (chassisData.volt * current) / 1'000'000.0f;
    // we're multiplying by 1'000'000.0f to convert from microwatts to watts

    const float dt = tap::arch::clock::getTimeMilliseconds() - prevTime;
    prevTime = tap::arch::clock::getTimeMilliseconds();
    energyBuffer -= ((consumedPower * safetyFactor) - chassisData.powerConsumptionLimit) * dt / 1000.0f;

    if (robotData.robotDataReceivedTimestamp != prevRobotDataReceivedTimestamp) {
        energyBuffer = chassisData.powerBuffer;
        prevRobotDataReceivedTimestamp = robotData.robotDataReceivedTimestamp;
    }

    energyBufferDisplay = chassisData.power;

    consumedPower = newChassisPower;
}

}  // namespace src::Utils::Control::PowerLimiting