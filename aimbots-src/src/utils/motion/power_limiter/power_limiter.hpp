#pragma once

#include "drivers.hpp"
#include "utils/common_types.hpp"

namespace src::utils::Control::PowerLimiting {

class PowerLimiter {
   public:
    PowerLimiter(const src::Drivers* drivers,
                 float startingEnergyBuffer,
                 float energyBufferLimitThreshold,
                 float energyBufferCritThreshold,
                 float safetyFactor);

    float getPowerLimitRatio();

   private:
    const src::Drivers* drivers;
    float startingEnergyBuffer;
    float energyBufferLimitThreshold;
    float energyBufferCritThreshold;

    float energyBuffer;
    float consumedPower;
    float safetyFactor;
    uint32_t prevTime;
    uint32_t prevRobotDataReceivedTimestamp;

    void updatePowerAndEnergyBuffer();
};

};  // namespace src::utils::Control::PowerLimiting