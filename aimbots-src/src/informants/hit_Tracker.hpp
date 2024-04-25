//ref helper and gets updated at driver level
#pragma once


//#include <tap/algorithms/contiguous_float.hpp>
//#include "transformers/robot_frames.hpp"
#include "utils/common_types.hpp"
//#include "subsystems/gimbal/gimbal.hpp"


//#include "drivers.hpp"


namespace src {
class Drivers;
} //nam


namespace src::Gimbal {
class GimbalSubsystem;
}




using namespace src::Utils;


namespace src::Informants {
   
class HitTracker{
public:
    HitTracker(src::Drivers* drivers);
    ~HitTracker() = default;
    //src::Informants::Transformers::RobotFrames& getRobotFrames() { return robotFrames; }


    void regSubsystems(
        src::Gimbal::GimbalSubsystem* gimbalSubsystem) {
        this->gimbalSubsystem = gimbalSubsystem;
    }


    void initalize();


    uint8_t getHitPanelID();
    //DAAG Continue to move getters with drivers to the cpp
    uint16_t getPrevHp();
   
    uint16_t getCurrHP();
    uint32_t getDataTimeStamp();


    bool wasHit();


    //returns hit angle relative to chassis front as 0
    float getHitAngle_chassisRelative();


    //returns hit angle relative to gimbal front as 0
    float getHitAngle_gimbalRelative();




private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbalSubsystem;
    //src::Informants::Transformers::RobotFrames robotFrames;
};
}
