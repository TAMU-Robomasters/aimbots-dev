#pragma once

#include "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/gpio/pwm.hpp"  //maybe not
#include "utils/common_types.hpp"
#include <string>

enum LimitSwitchState {
    PRESSED = 1,
    RELEASED = 0,
};

using C6 = Board::DigitalInPinC6;
using C7 = Board::DigitalInPinC7;

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

    //optional
    int counter;
    const EdgeType edge; //which edge to count with in refresh
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
