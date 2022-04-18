#pragma once

// #include "tap/communication/gpio/pwm.hpp"
//
#include "drivers.hpp"
#include "tap/communication/gpio/digital.hpp"  //maybe not
#include "utils/common_types.hpp"

namespace utils {

enum LimitSwitchState {
    PRESSED = 1,
    RELEASED = 0,
};

class LimitSwitch {
   public:
    LimitSwitch(src::Drivers* drivers, InputPins rxPin);

    bool readSwitch();

    void updateSwitch();

    bool isRising() const;
    bool isFalling() const;
    bool isPressed() const;
    bool isReleased() const;

   private:
    InputPins rxPin;
    src::Drivers* drivers;
    LimitSwitchState currSwitchState;
    LimitSwitchState prevSwitchState;

    bool isStateChanged(bool currentState);
};

}  // namespace utils