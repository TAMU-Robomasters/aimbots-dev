#include "ref_helper_turreted.hpp"

#include "utils/robot_specific_inc.hpp"

namespace src::Utils {

RefereeHelperTurreted::RefereeHelperTurreted(src::Drivers* drivers, BarrelID& barrelID)
    : RefereeHelperInterface(drivers),
      currBarrelID(barrelID)  //
{}

std::optional<uint16_t> RefereeHelperTurreted::getCurrBarrelProjectileSpeedLimit() {
    auto& turretData = drivers->refSerial.getRobotData().turret;

    switch (currBarrelID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1:
            return turretData.barrelSpeedLimit17ID1;

        case RefSerialRxData::MechanismID::TURRET_17MM_2:
            return turretData.barrelSpeedLimit17ID2;

        case RefSerialRxData::MechanismID::TURRET_42MM:
            return turretData.barrelSpeedLimit42;

        default:
            return std::nullopt;
    }
}

std::optional<uint16_t> RefereeHelperTurreted::getProjectileSpeedLimit(BarrelID barrelID) {
    auto& turretData = drivers->refSerial.getRobotData().turret;

    switch (barrelID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1:
            return turretData.barrelSpeedLimit17ID1;

        case RefSerialRxData::MechanismID::TURRET_17MM_2:
            return turretData.barrelSpeedLimit17ID2;

        case RefSerialRxData::MechanismID::TURRET_42MM:
            return turretData.barrelSpeedLimit42;

        default:
            return std::nullopt;
    }
}

float RefereeHelperTurreted::getLastProjectileSpeed() {
    auto& turretData = drivers->refSerial.getRobotData().turret;
    return turretData.bulletSpeed;
}

void RefereeHelperTurreted::updatePredictedProjectileSpeed() {
    auto& turretData = drivers->refSerial.getRobotData().turret;

    auto projectileSpeedLimit = this->getCurrBarrelProjectileSpeedLimit();
    if (projectileSpeedLimit.has_value()) {
        projectileLaunchSpeedPredictor.updatePredictedLaunchSpeed(
            turretData.bulletSpeed,
            projectileSpeedLimit.value(),
            turretData.lastReceivedLaunchingInfoTimestamp);
    }
}

bool RefereeHelperTurreted::isCurrBarrelHeatUnderLimit(float percentageOfLimit) {
    auto& turretData = drivers->refSerial.getRobotData().turret;

    uint16_t lastHeat = 0;
    uint16_t heatLimit = 0;

    switch (currBarrelID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1: {
            lastHeat = turretData.heat17ID1;
            heatLimit = turretData.heatLimit17ID1;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_17MM_2: {
            lastHeat = turretData.heat17ID2;
            heatLimit = turretData.heatLimit17ID2;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_42MM: {
            lastHeat = turretData.heat42;
            heatLimit = turretData.heatLimit42;
            break;
        }
        default:
            return true;
    }

    return (lastHeat <= (static_cast<float>(heatLimit) * percentageOfLimit));
}

bool RefereeHelperTurreted::isBarrelHeatUnderLimit(float percentageOfLimit, BarrelID barrelID) {
    auto turretData = drivers->refSerial.getRobotData().turret;

    uint16_t lastHeat = 0;
    uint16_t heatLimit = 0;

    switch (barrelID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1: {
            lastHeat = turretData.heat17ID1;
            heatLimit = turretData.heatLimit17ID1;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_17MM_2: {
            lastHeat = turretData.heat17ID2;
            heatLimit = turretData.heatLimit17ID2;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_42MM: {
            lastHeat = turretData.heat42;
            heatLimit = turretData.heatLimit42;
            break;
        }
        default:
            return true;
    }

    return (lastHeat <= (static_cast<float>(heatLimit) * percentageOfLimit));
}

}  // namespace src::Utils
