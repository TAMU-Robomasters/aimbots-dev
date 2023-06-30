#pragma once

#include "utils/common_types.hpp"

#include "projectile_launch_speed_predictor.hpp"
#include "ref_helper_interface.hpp"

using RefSerialRxData = tap::communication::serial::RefSerialData::Rx;

namespace src::Utils {

enum BarrelSpeeds : uint8_t {
    SPEED15_MS = 0,
    SPEED18_MS = 1,
    SPEED30_MS = 2,
};

class RefereeHelperTurreted : public RefereeHelperInterface {
public:
    RefereeHelperTurreted(src::Drivers*, BarrelID& barrelID);
    ~RefereeHelperTurreted() = default;

    std::optional<uint16_t> getCurrBarrelProjectileSpeedLimit();
    std::optional<uint16_t> getProjectileSpeedLimit(BarrelID barrelID);

    float getLastProjectileSpeed();

    void updatePredictedProjectileSpeed();

    inline std::optional<float> getPredictedProjectileSpeed() {
        return projectileLaunchSpeedPredictor.getPredictedLaunchSpeed();
    }

    bool isCurrBarrelHeatUnderLimit(float percentageOfLimit);
    bool isBarrelHeatUnderLimit(float percentageOfLimit, BarrelID barrelID);
    bool canCurrBarrelShootSafely();

    BarrelID getCurrentBarrel() { return currBarrelID; }
    void setCurrentBarrel(BarrelID barrelID) { currBarrelID = barrelID; }

private:
    BarrelID& currBarrelID;

    src::Utils::ProjectileLaunchSpeedPredictor<PROJECTILE_SPEED_QUEUE_SIZE> projectileLaunchSpeedPredictor;

    int heatGainedPerProjectile[2] = {10,100}; //{17mm, 42mm}
};

}  // namespace src::Utils
