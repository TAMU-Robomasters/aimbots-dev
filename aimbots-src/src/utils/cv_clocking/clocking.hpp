#pragma once

#include "drivers.hpp"

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
    private:
        src::Drivers* drivers;
};
}