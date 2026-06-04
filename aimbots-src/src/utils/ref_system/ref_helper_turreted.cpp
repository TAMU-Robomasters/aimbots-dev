#include "ref_helper_turreted.hpp"

#include "utils/tools/robot_specific_defines.hpp"
#include "subsystems/gimbal/gimbal_constants.hpp"
#include "subsystems/chassis/chassis_constants.hpp"

namespace src::Utils {

RefereeHelperTurreted::RefereeHelperTurreted(src::Drivers* drivers, BarrelID& barrelID, int safetyHeatTolerance = 0)
    : RefereeHelperInterface(drivers),
      currBarrelID(barrelID),
      safetyHeatTolerance(safetyHeatTolerance)  //
{}

float RefereeHelperTurreted::getReceivedDPS() {
    auto botData = drivers->refSerial.getRobotData();
    return botData.receivedDps;
}

float RefereeHelperTurreted::getCurrHealthPercentage() {
    auto botData = drivers->refSerial.getRobotData();
    return static_cast<float>(botData.currentHp) / static_cast<float>(botData.maxHp);
}

std::optional<uint16_t> RefereeHelperTurreted::getCurrBarrelProjectileSpeedLimit() {
    switch (currBarrelID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1:
            return RefSerialRxData::MAX_LAUNCH_SPEED_17MM;

        case RefSerialRxData::MechanismID::TURRET_17MM_2:
            return RefSerialRxData::MAX_LAUNCH_SPEED_17MM;

        case RefSerialRxData::MechanismID::TURRET_42MM:
            return RefSerialRxData::MAX_LAUNCH_SPEED_42MM;

        default:
            return std::nullopt;
    }
}

std::optional<uint16_t> RefereeHelperTurreted::getProjectileSpeedLimit(BarrelID barrelID) {
    switch (barrelID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1:
            return RefSerialRxData::MAX_LAUNCH_SPEED_17MM;

        case RefSerialRxData::MechanismID::TURRET_17MM_2:
            return RefSerialRxData::MAX_LAUNCH_SPEED_17MM;

        case RefSerialRxData::MechanismID::TURRET_42MM:
            return RefSerialRxData::MAX_LAUNCH_SPEED_42MM;

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

    heatLimit = turretData.heatLimit;

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
            return true;
    }

    return (lastHeat <= (static_cast<float>(heatLimit) * percentageOfLimit));
}

bool RefereeHelperTurreted::isBarrelHeatUnderLimit(float percentageOfLimit, BarrelID barrelID) {
    auto turretData = drivers->refSerial.getRobotData().turret;

    uint16_t lastHeat = 0;
    uint16_t heatLimit = 0;

    heatLimit = turretData.heatLimit;

    switch (barrelID) {
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
            return true;
    }

    return (lastHeat <= (static_cast<float>(heatLimit) * percentageOfLimit));
}

bool RefereeHelperTurreted::canCurrBarrelShootSafely() {
    auto& turretData = drivers->refSerial.getRobotData().turret;

    uint16_t lastHeat = 0;
    uint16_t heatLimit = 0;
    auto projectileType = RefSerialRxData::BulletType::AMMO_17;

    heatLimit = turretData.heatLimit;

    switch (currBarrelID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1: {
            lastHeat = turretData.heat17ID1;
            projectileType = RefSerialRxData::AMMO_17;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_17MM_2: {
            lastHeat = turretData.heat17ID2;
            projectileType = RefSerialRxData::AMMO_17;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_42MM: {
            lastHeat = turretData.heat42;
            projectileType = RefSerialRxData::AMMO_42;
            break;
        }
        default:
            return true;
    }

    return ((lastHeat + heatGainedPerProjectile[projectileType - 1] + safetyHeatTolerance) < heatLimit) /*||
           (heatGainedPerProjectile[projectileType - 1] >= heatLimit)*/
        ;  //-1 is to align array index with enum values
}

bool RefereeHelperTurreted::canSpecificBarrelShootSafely(BarrelID barrelID) {
    auto& turretData = drivers->refSerial.getRobotData().turret;

    uint16_t lastHeat = 0;
    uint16_t heatLimit = 0;
    auto projectileType = RefSerialRxData::BulletType::AMMO_17;

    heatLimit = turretData.heatLimit;

    switch (barrelID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1: {
            lastHeat = turretData.heat17ID1;
            projectileType = RefSerialRxData::AMMO_17;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_17MM_2: {
            lastHeat = turretData.heat17ID2;
            projectileType = RefSerialRxData::AMMO_17;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_42MM: {
            lastHeat = turretData.heat42;
            projectileType = RefSerialRxData::AMMO_42;
            break;
        }

        default:
            return true;
    }

    return (lastHeat + heatGainedPerProjectile[projectileType - 1] < heatLimit) ||
           (heatGainedPerProjectile[projectileType - 1] >= heatLimit);  //-1 is to align array index with enum values
}

uint8_t RefereeHelperTurreted::getRemainingProjectiles() {
    auto projectileType = RefSerialRxData::BulletType::AMMO_17;

    switch (currBarrelID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1: {
            projectileType = RefSerialRxData::AMMO_17;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_17MM_2: {
            projectileType = RefSerialRxData::AMMO_17;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_42MM: {
            projectileType = RefSerialRxData::AMMO_42;
            break;
        }

        default:
            return true;
    }
    uint16_t remainingHeat = getCurrBarrelLimit() - getCurrBarrelHeat();
    return remainingHeat / heatGainedPerProjectile[projectileType - 1];
}

int projectileAllowanceDisplay = 0;
float maxRotationsDisplay = 0;

uint64_t RefereeHelperTurreted::getAllowableFeederRotation(int projectileBuffer) {
    // Calculating the remaining projectiles that can be shot based on heat
    uint8_t projectilesRemaining = getRemainingProjectiles() - projectileBuffer;

    projectileAllowanceDisplay = projectilesRemaining;
    // get the maximum rotations the feeder can make based on how many projectiles
    // it shoots in one rotation
    float maxRotations = ((float)projectilesRemaining) / PROJECTILES_PER_FEEDER_ROTATION;
    maxRotationsDisplay = maxRotations;
    // Get the maximum absolute position the motor can get to
    int64_t encoderChangeThreshold = DJIMotorEncoder::ENC_RESOLUTION * maxRotations;
    return encoderChangeThreshold;
}

}  // namespace src::Utils
