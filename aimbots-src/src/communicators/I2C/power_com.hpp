//for reading the power limiting board for robots
//TODO fix targets, currently just not defined for engineer
#ifndef TARGET_ENGINEER
#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"
#include "modm/processing/resumable.hpp"

namespace src::Communication {
    class PowerComm : public tap::communication::serial::DJISerial{
        public:
            
    }
}



#endif