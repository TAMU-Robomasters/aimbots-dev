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
    RefereeHelperTurreted(src::Drivers*, BarrelID& barrelID, int safetyHeatTolerance);
    ~RefereeHelperTurreted() = default;

    float getReceivedDPS();

    float getCurrHealthPercentage();

    std::optional<uint16_t> getCurrBarrelProjectileSpeedLimit();
    std::optional<uint16_t> getProjectileSpeedLimit(BarrelID barrelID);

    float getLastProjectileSpeed();

    // Needs to be called frequently to catch each projectile launch
    void updatePredictedProjectileSpeed();

    inline std::optional<float> getPredictedProjectileSpeed() {
        return projectileLaunchSpeedPredictor.getPredictedLaunchSpeed();
    }

    bool isCurrBarrelHeatUnderLimit(float percentageOfLimit);
    bool isBarrelHeatUnderLimit(float percentageOfLimit, BarrelID barrelID);
    bool canCurrBarrelShootSafely();
    bool canSpecificBarrelShootSafely(BarrelID barrelID);

    uint16_t getCurrBarrelHeat() {
        auto& turretData = drivers->refSerial.getRobotData().turret;

        uint16_t lastHeat = 0;

        switch (currBarrelID) {
            case RefSerialRxData::MechanismID::TURRET_17MM_1: {
                lastHeat = turretData.heat17ID1;
                break;
            }
            case RefSerialRxData::MechanismID::TURRET_17MM_2: {
                lastHeat = turretData.heat17ID2;
                break;
            }
            case RefSerialRxData::MechanismID::TURRET_42MM: {
                lastHeat = turretData.heat42;
                break;
            }
            default:
                return 1;
        }
        return lastHeat;
    }

    uint16_t getCurrBarrelLimit() {
        auto& turretData = drivers->refSerial.getRobotData().turret;

        uint16_t heatLimit = 0;

        switch (currBarrelID) {
            case RefSerialRxData::MechanismID::TURRET_17MM_1: {
                heatLimit = turretData.heatLimit17ID1;
                break;
            }
            case RefSerialRxData::MechanismID::TURRET_17MM_2: {
                heatLimit = turretData.heatLimit17ID2;
                break;
            }
            case RefSerialRxData::MechanismID::TURRET_42MM: {
                heatLimit = turretData.heatLimit42;
                break;
            }
            default:
                return 1;
        }
        return heatLimit;
    }

    BarrelID getCurrentBarrel() { return currBarrelID; }
    void setCurrentBarrel(BarrelID barrelID) { currBarrelID = barrelID; }

private:
    BarrelID& currBarrelID;

    src::Utils::ProjectileLaunchSpeedPredictor<PROJECTILE_SPEED_QUEUE_SIZE> projectileLaunchSpeedPredictor;

    int heatGainedPerProjectile[2] = {10, 100};  //{17mm, 42mm}

    int safetyHeatTolerance;
};

}  // namespace src::Utils
