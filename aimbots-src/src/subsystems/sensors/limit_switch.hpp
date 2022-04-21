#pragma once

#include "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/gpio/pwm.hpp"  //maybe not
#include "utils/common_types.hpp"

enum LimitSwitchState {
    PRESSED = 1,
    RELEASED = 0,
};


enum EdgeType {
    FALLING = 0,
    RISING = 1,
    // NONE = 2, //So we can return if it is neither at a rising/falling point
};


class LimitSwitch : public tap::control::Subsystem {
private:
    InputPins rxPin;
    src::Drivers* drivers;
    LimitSwitchState currSwitchState;
    LimitSwitchState prevSwitchState;

    //optional
    int counter;
    const EdgeType edge; //which edge to count with in refresh
public:
    LimitSwitch(src::Drivers* drivers, InputPins rxPin, EdgeType edge);

    void initialize() override;
    void refresh() override;
    //
    bool readSwitch();
    void updateSwitch();
    bool isRising() const;
    bool isFalling() const;
    bool isStateChanged() const;
    int getCurrentCount(int n) const;
};
