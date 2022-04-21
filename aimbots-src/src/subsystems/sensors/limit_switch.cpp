#include "subsystems/sensors/limit_switch.hpp"

LimitSwitch::LimitSwitch(src::Drivers* drivers, InputPins rxPin, EdgeType edge)
    : Subsystem(drivers),
    rxPin(rxPin),
    counter(0),
    edge(edge) {};

void LimitSwitch::initialize() { //awesome
    drivers->digital.configureInputPullMode(rxPin, modm::platform::Gpio::InputType::PullUp);
}

int counter_debug = 0;
void LimitSwitch::refresh() {
    updateSwitch();
    if (edge == RISING) { // == EdgeType::RISING == 1
        counter += (isRising() ? 1 : 0);
        //counter = counter  % mod
    } 
    else {
        counter += (isFalling() ? 1 : 0);
    }

    counter_debug = counter;
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

int state = 0;
void LimitSwitch::updateSwitch() {
    prevSwitchState = currSwitchState;
    currSwitchState = static_cast<LimitSwitchState>(readSwitch());
    state = static_cast<int>(currSwitchState);
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
