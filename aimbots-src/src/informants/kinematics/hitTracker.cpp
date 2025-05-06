#include "hitTracker.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

// namespace src {
// class Drivers;
// }

namespace src::Informants {

HitTracker::HitTracker(src::Drivers* drivers) : drivers(drivers) {}

void HitTracker::initalize() { hitTimer.stop(); }

uint8_t HitTracker::getHitPanelID() { return static_cast<uint8_t>(this->drivers->refSerial.getRobotData().damagedArmorId); };

uint16_t HitTracker::getPrevHP() { return this->drivers->refSerial.getRobotData().previousHp; };

uint16_t HitTracker::getCurrHP() { return this->drivers->refSerial.getRobotData().currentHp; };

uint32_t HitTracker::getDataTimeStamp() { return this->drivers->refSerial.getRobotData().robotDataReceivedTimestamp; };

bool HitTracker::wasHit() {
    if (this->drivers->refSerial.getRobotData().receivedDps != 0) {
        return true;
    } else {
        return false;
    }
};

bool HitTracker::recentlyHit() {
    if (wasHit()) {
        hitTimer.restart(HIT_EXPIRE_TIME);
    }
    return !hitTimer.isExpired();
};

float HitTracker::getHitAngle_chassisRelative() {
    // get armor panel hit
    if (!recentlyHit()) {
        return 69.0f;
    }
    uint8_t panel = getHitPanelID();
    float hitAngle = 0.0f;
    switch (panel) {
        case 0:  // Front
            hitAngle = 0.0f;
            break;
        case 1:  // Left
            hitAngle = M_PI / 2.0f;
            break;
        case 2:  // Rear
            hitAngle = M_PI;
            break;
        case 3:  // Right
            hitAngle = -M_PI / 2.0f;
            break;
        default:
            break;
    }
    return hitAngle;
}

float HitTracker::getHitAngle_gimbalRelative() {
    // get chassisRelative angle?
    float chassis_hitAngle = getHitAngle_chassisRelative();
    if (chassis_hitAngle == 69.0f) {
        return 69.0f;
    }
    // get angle btwn gimbal-chassis?
    float gimbalAngle = drivers->kinematicInformant.getFieldGimbal()
                            ->getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat()
                            .getWrappedValue();
    // calc and return
    WrappedFloat hitAngle = WrappedFloat(gimbalAngle + chassis_hitAngle, -M_PI, M_PI);
    return hitAngle.getWrappedValue();
}
}  // namespace src::Informants

// should be object that acesses robot data through
