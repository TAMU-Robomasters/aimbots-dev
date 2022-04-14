#pragma once

// #include "tap/communication/gpio/pwm.hpp"
// 
#include "utils/common_types.hpp"
#include "<tap/communication/gpio/digital.hpp>" //rip digital

namespace utils {

class LimitSwitch {
        public:
            LimitSwitch(InputPins rxPin)
                :  rxPin(rxPin) {}
            bool updateSwitch();
            bool readSwitch();

        private:
            InputPins rxPin;

            bool previousState;

            bool isStateChanged(bool currentState);

};

} // namespace utils