#pragma once

#include "utils/common_types.hpp"
#include "utils/filters/ema.hpp"

#include "drivers.hpp"

using RefSerialRxData = tap::communication::serial::RefSerialData::Rx;

namespace src::Utils {

class RefereeHelper {
public:
    RefereeHelper(src::Drivers*, float bulletSpeedFilterAlpha = 0.5f);
    ~RefereeHelper() = default;

    uint16_t getProjectileSpeedLimit();
    uint16_t getProjectileSpeedLimit(BarrelID barrelID);

    float getLastProjectileSpeed();
    float getPredictedProjectileSpeed();

    float getPredictedProjectileSpeed(BarrelID barrelID);

    bool isBarrelHeatUnderLimit(float percentageOfLimit);
    bool isBarrelHeatUnderLimit(float percentageOfLimit, BarrelID barrelID);

    int16_t getCurrentBarrel();

    RefSerialRxData::GameStage getGameStage();

private:
    src::Drivers* drivers;
    src::Utils::Filters::EMAFilter bulletSpeedFilter;
};

}  // namespace src::Utils
