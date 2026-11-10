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
        void togglePWM();
    private:
        src::Drivers* drivers;
        MilliTimeout resetTimer;
};
}