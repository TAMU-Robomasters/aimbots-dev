#pragma once

#include "utils/tools/common_types.hpp"
namespace src {
class Drivers;
}

namespace utils::CV_Clocking {
    
class CVClocker {
    public:
        CVClocker(src::Drivers* drivers);
        ~CVClocker() = default;

        void reset();
        void startSignal();
        void stopSignal();
        void forceHigh();
  //      void togglePWM();
        void togglePWM(bool fast);
        //toggles pin PE13 and keeps track of ticks to jetson TODO: make doc of what pins are used for what
        void toogleClock();
        uint32_t getTicks();
    private:
        src::Drivers* drivers;
        MilliTimeout resetTimer;
        uint64_t ticks;
};
}