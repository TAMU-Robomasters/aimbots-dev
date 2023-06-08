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
    float getLastProjectileSpeed();
    float getPredictedProjectileSpeed();

    bool isBarrelHeatUnderLimit(float percentageOfLimit);

    RefSerialRxData::GameStage getGameStage();

private:
    src::Drivers* drivers;
    EMAFilter bulletSpeedFilter;
};

}  // namespace src::Utils
