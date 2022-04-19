#include "utils/limit_switch.hpp"

LimitSwitch::LimitSwitch(src::Drivers* drivers, InputPins rxPin, EdgeType edge)
    : Subsystem(drivers),
    rxPin(rxPin),
    counter(0),
    edge(static_cast<EdgeType>(edge)) {};

void LimitSwitch::initialize() { //awesome
}

void LimitSwitch::refresh() {
    updateSwitch();
    if (edge) { // == EdgeType::RISING == 1
        counter += (isRising() ? 1 : 0);
        //counter = counter  % mod
    } 
    else {
        counter += (isFalling() ? 1 : 0);
    }
//   counter += (edge ? limitSwitch.isRising() ? 1 : 0 : limitSwitch.isFalling() ? 1 : 0);
//uncomment this line ^^ to run the code faster
    /*
    if (counter==mod) {
        //DO event
        counter =0;
    }
    */
}

bool LimitSwitch::readSwitch() {
    return drivers->digital.read(rxPin);
}

void LimitSwitch::updateSwitch() {
    prevSwitchState = currSwitchState;

    currSwitchState = static_cast<LimitSwitchState>(readSwitch());
}

bool LimitSwitch::isRising() const {
    return (currSwitchState == LimitSwitchState::PRESSED) && (prevSwitchState == LimitSwitchState::RELEASED);
}

bool LimitSwitch::isFalling() const {
    return (currSwitchState == LimitSwitchState::RELEASED) && (prevSwitchState == LimitSwitchState::PRESSED);
}

bool LimitSwitch::isStateChanged() const {
    return (currSwitchState != prevSwitchState);
}
