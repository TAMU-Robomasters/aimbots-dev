//should be object that acesses robot data through
//ref helper and gets updated at driver level
#pragma once

#include "utils/common_types.hpp"
#include "subsystems/gimbal/gimbal.hpp"

#include "drivers.hpp"

namespace src {
class Drivers;
}

namespace src::Gimbal {
class GimbalSubsystem;
}


using RefSerialRxData = tap::communication::serial::RefSerialData::Rx;
using namespace src::Utils;

namespace src::Informants {
class HitTracker{
public:
    HitTracker(src::Drivers* drivers);
    ~HitTracker() = default;
    src::Informants::Transformers::RobotFrames& getRobotFrames() { return robotFrames; }

    RefSerialRxData::ArmorId getHitPanelID() {
        auto robotData = drivers->refSerial.getRobotData();
        return robotData.damagedArmorId;
    }
    uint16_t getPrevHp(){
        auto robotData = drivers->refSerial.getRobotData();
        return robotData.previousHp;
    }
    uint16_t getCurrHP(){
        auto robotData = drivers->refSerial.getRobotData();
        return robotData.currentHp;
    }
    uint32_t getDataTimeStamp(){
        auto robotData = drivers->refSerial.getRobotData();
        return robotData.robotDataReceivedTimestamp;
    }

    bool wasHit();

    //returns hit angle relative to chassis front as 0
    float getHitAngle_chassisRelative();

    //returns hit angle relative to gimbal front as 0
    float getHitAngle_gimbalRelative();


    private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbal;
    src::Informants::Transformers::RobotFrames robotFrames;
};
}

