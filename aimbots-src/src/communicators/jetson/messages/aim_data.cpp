#include <cstdint>
#include "communicators/jetson/messages/aim_data.hpp"

#ifdef GIMBAL_COMPATIBLE

using namespace src::Communication; 

AimData::AimData(src::Drivers* drivers, uint8_t messageID) 
    : DataFromJetson<AimMessage>(drivers, messageID)
{}

#endif // #ifdef GIMBAL_COMPATIBLE
