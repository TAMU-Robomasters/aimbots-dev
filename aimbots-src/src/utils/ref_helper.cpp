#include "ref_helper.hpp"

namespace src::Utils {

RefereeHelper::RefereeHelper(src::Drivers* drivers, float bulletSpeedFilterAlpha)
    : drivers(drivers),
      bulletSpeedFilter(EMAFilter(bulletSpeedFilterAlpha))  //
{}

uint16_t RefereeHelper::getProjectileSpeedLimit() {
    auto refSysRobotTurretData = drivers->refSerial.getRobotData().turret;
    auto launcherID = refSysRobotTurretData.launchMechanismID;

    float refSpeedLimit = 0;

    switch (launcherID) {  // gets launcher ID from ref serial, sets speed limit accordingly
        case RefSerialRxData::MechanismID::TURRET_17MM_1: {
            refSpeedLimit = refSysRobotTurretData.barrelSpeedLimit17ID1;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_17MM_2: {
            refSpeedLimit = refSysRobotTurretData.barrelSpeedLimit17ID2;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_42MM: {
            refSpeedLimit = refSysRobotTurretData.barrelSpeedLimit42;
            break;
        }
        default:
            break;
    }

    return refSpeedLimit;
}

float RefereeHelper::getLastProjectileSpeed() {
    auto refSysRobotTurretData = drivers->refSerial.getRobotData().turret;
    return refSysRobotTurretData.bulletSpeed;
}

float RefereeHelper::getPredictedProjectileSpeed() {
    auto refSysRobotTurretData = drivers->refSerial.getRobotData().turret;

    // if it's been more than 5 seconds since the last projectile launch or last projectile launch hasn't been updated,
    // assume the next one will be at the speed limit
    if (refSysRobotTurretData.lastReceivedLaunchingInfoTimestamp == 0 ||
        tap::arch::clock::getTimeMilliseconds() > refSysRobotTurretData.lastReceivedLaunchingInfoTimestamp + 5'000) {
        return static_cast<float>(getProjectileSpeedLimit());
    }

    return getLastProjectileSpeed();  // for now, just send the last projectile speed
}

bool RefereeHelper::isBarrelHeatUnderLimit(float percentageOfLimit) {
    auto turretData = drivers->refSerial.getRobotData().turret;

    uint16_t lastHeat = 0;
    uint16_t heatLimit = 0;

    auto launcherID = turretData.launchMechanismID;
    switch (launcherID) {
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
            break;
    }

    return (lastHeat <= (static_cast<float>(heatLimit) * percentageOfLimit));
}

RefSerialRxData::GameStage RefereeHelper::getGameStage() {
    auto gameData = drivers->refSerial.getGameData();
    return gameData.gameStage;
}

}  // namespace src::Utils
