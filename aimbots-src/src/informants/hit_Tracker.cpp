#include "hit_Tracker.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "drivers.hpp"

namespace src::Informants {
    
HitTracker::HitTracker(src::Drivers* drivers)
    : drivers(drivers)
{
}

void HitTracker::initialize() {
}


bool HitTracker::wasHit(){
    return (getPrevHp() > getCurrHP());
}

float HitTracker::getHitAngle_chassisRelative(){
    //get armor panel hit
    RefSerialRxData::ArmorId panel = getHitPanelID();
    float hitAngle = 0.0f;
    switch(panel) {
        case RefSerialRxData::ArmorId::FRONT:
            hitAngle = 0.0f;
            break;
        case RefSerialRxData::ArmorId::LEFT:
            hitAngle = M_PI / 2.0f;
            break;
        case RefSerialRxData::ArmorId::REAR:
            hitAngle = M_PI;
            break;
        case RefSerialRxData::ArmorId::RIGHT:
            hitAngle = -M_PI / 2.0f;
            break;
        default:
            break;
    }
    return hitAngle;
}

float HitTracker::getHitAngle_gimbalRelative(){
    //get chassisRelative angle?
    float chassis_hitAngle = getHitAngle_chassisRelative();
    //get angle btwn gimbal-chassis?
    float gimbalAngle = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
    //calc and return
    return gimbalAngle + chassis_hitAngle;
}
}