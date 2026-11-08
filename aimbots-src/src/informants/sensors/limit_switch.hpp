#pragma once

#include <string>

#include "drivers.hpp"
#include "tap/communication/gpio/pwm.hpp"  //maybe not
#include "tap/control/subsystem.hpp"
#include "utils/tools/common_types.hpp"

namespace src::Informants {

enum LimitSwitchState {
    PRESSED = 1,
    RELEASED = 0,
};

// DM-MC02: the PWM2 header pin (PA2) is used as a GPIO input. It is left out of
// taproot:board:pwm_pins in project.xml. The Type C used C6/C7.
using A2 = modm::platform::GpioA2;

enum EdgeType {
    FALLING = 0,
    RISING = 1,
    // NONE = 2, //So we can return if it is neither at a rising/falling point
};

class LimitSwitch {
   private:
    std::string rxPin;
    src::Drivers* drivers;
    LimitSwitchState currSwitchState;
    LimitSwitchState prevSwitchState;

    // optional
    int counter;
    const EdgeType edge;  // which edge to count with in refresh
   public:
    LimitSwitch(std::string rxPin, EdgeType edge);

    void initialize();
    void refresh();
    //
    bool readSwitch();
    void updateSwitch();
    bool isRising() const;
    bool isFalling() const;
    bool isStateChanged() const;
    int getCurrentCount() const;
};

}  // namespace src::Informants