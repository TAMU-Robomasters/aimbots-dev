#include "informants/limit_switch.hpp"

LimitSwitch::LimitSwitch(std::string rxPin, EdgeType edge)
    : rxPin(rxPin),
      counter(0),
      edge(edge){};

void LimitSwitch::initialize() {  // awesome
    C6::configure(modm::platform::Gpio::InputType::PullDown);
    C7::configure(modm::platform::Gpio::InputType::PullDown);
}

int counter_debug = 0;
bool debug_C7_direct = false;
LimitSwitchState debug_C6_direct = static_cast<LimitSwitchState>(1);
void LimitSwitch::refresh() {
    updateSwitch();
    if (edge == RISING) {  // == EdgeType::RISING == 1
        counter += (isRising() ? 1 : 0);
        // counter = counter  % mod
    } else {
        counter += (isFalling() ? 1 : 0);
    }
    // debug
    counter_debug = counter;
    // switch_debug = static_cast<LimitSwitchState>(readSwitch());
    debug_C7_direct = readSwitch();
    //   counter += (edge ? limitSwitch.isRising() ? 1 : 0 : limitSwitch.isFalling() ? 1 : 0);
    // uncomment this line ^^ to run the code faster
}

bool LimitSwitch::readSwitch() {
    // return drivers->digital.read(rxPin);
    debug_C6_direct = static_cast<LimitSwitchState>(Board::DigitalInPinC6::read());
    if (rxPin == "C6")
        return C6::read();
    else if (rxPin == "C7")
        return C7::read();
    return true;
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

int LimitSwitch::getCurrentCount() const {
    return counter;
}