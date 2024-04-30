#include "hit_Tracker.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "drivers.hpp"


// namespace src {
// class Drivers;
// }




namespace src::Informants {
   
HitTracker::HitTracker(src::Drivers* drivers)
    : drivers(drivers)
{
}


void HitTracker::initalize(){
}


uint8_t HitTracker::getHitPanelID() {
    return static_cast <uint8_t> (this->drivers->refSerial.getRobotData().damagedArmorId);
};


uint16_t HitTracker::getPrevHp(){
        return this->drivers->refSerial.getRobotData().previousHp;
};


uint16_t HitTracker::getCurrHP(){
        return this->drivers->refSerial.getRobotData().currentHp;
};


uint32_t HitTracker::getDataTimeStamp(){
        return this->drivers->refSerial.getRobotData().robotDataReceivedTimestamp;
};


bool HitTracker::wasHit(){
    return (getPrevHp() > getCurrHP());
};


float HitTracker::getHitAngle_chassisRelative(){
    //get armor panel hit
    uint8_t panel = getHitPanelID();
    float hitAngle = 0.0f;
    switch(panel) {
        case 0://Front
            hitAngle = 0.0f;
            break;
        case 1://Left
            hitAngle = M_PI / 2.0f;
            break;
        case 2://Rear
            hitAngle = M_PI;
            break;
        case 3://Right
            hitAngle = -M_PI / 2.0f;
            break;
        default:
            break;
    }
    return hitAngle;
}


ContiguousFloat HitTracker::getHitAngle_gimbalRelative(){
    //get chassisRelative angle?
    float chassis_hitAngle = this->drivers->hitTracker.getHitAngle_chassisRelative();
    //get angle btwn gimbal-chassis?
    float gimbalAngle = this->drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsContiguousFloat().getValue();
    //calc and return
    ContiguousFloat hitAngle = ContiguousFloat(gimbalAngle + chassis_hitAngle, -M_PI, M_PI);
    return hitAngle;
}
}



//should be object that acesses robot data through
