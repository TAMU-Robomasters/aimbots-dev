#pragma once

#include "drivers.hpp"

namespace utils::CV_Clocking {
    
    void reset(src::Drivers* drivers);
    void startSignal(src::Drivers* drivers);
    void stopSignal(src::Drivers* drivers);
}