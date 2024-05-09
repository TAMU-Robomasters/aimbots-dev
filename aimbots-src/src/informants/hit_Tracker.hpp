//ref helper and gets updated at driver level
#pragma once

#include <optional>

#include <tap/algorithms/contiguous_float.hpp>
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
    uint16_t getPrevHP();
   
    uint16_t getCurrHP();
    uint32_t getDataTimeStamp();


    bool wasHit();
    bool recentlyHit();


    //armor IDs:
    //   Front
//  _____ 0  ____
 // |   |___|   |	   
//	|___| | |___|	    
// 	 1    |    3 	  
//	 ___  |  ___ 	  
//  |   |_|_|   |	  
//	|___| 2 |___|
//       Back

//returns hit angle relative to chassis front as 0
//          
//        Front: 0 degrees
//          _______
//        |        |    
//   -pi |          |  pi
//       |          |
//        |________|

    float getHitAngle_chassisRelative();


    //returns hit angle relative to gimbal front as 0
    float getHitAngle_gimbalRelative();




private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbalSubsystem;
    uint16_t prevHP;
    //src::Informants::Transformers::RobotFrames robotFrames;
    static const uint32_t HIT_EXPIRE_TIME = 5000;

    MilliTimeout hitTimer;
};
}
