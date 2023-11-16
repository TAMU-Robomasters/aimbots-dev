#include "grabber.hpp"

#ifdef GRABBER_COMPATIBLE //TODO: Define this in a constants file later

namespace src::Grabber {

GrabberSubsystem::GrabberSubsystem(src::Drivers* drivers, tap::gpio::Pwm* pwm){

}
void GrabberSubsystem::initialize() {
    drivers->pwm.init();
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm DEFAULT_TIMER1_FREQUENCY, 2000);
}

void GrabberSubsystem::refresh() {

}

void GrabberSubsystem::activate(float duty_cycle) {
    if (duty_cycle > 1.0f) {
        duty_cycle = 1.0f;
    } else if (duty_cycle < 0.0f) {
        duty_cycle = 0.0f;
    }
    drivers->pwm.write(duty_cycle, GRABBER_PIN);
}

void GrabberSubsystem::end() {
    drivers->pwm.write(0.0f, GRABBER_PIN);
}



} //namespace src::Grabber


#endif //GRABBER_COMPATIBLE