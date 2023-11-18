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
    activate(1.0f);
    activate(0.0f);
}

void GrabberSubsystem::activate(float duty_cycle) {
    if (duty_cycle > 1.0f) {
        duty_cycle = 1.0f;
    } else if (duty_cycle < 0.0f) {
        duty_cycle = 0.0f;
    }
    drivers->pwm.write(duty_cycle, GRABBER_PIN);
}

// void GrabberSubsystem::deactivate(float duty_cycle) {
//     if (duty_cycle > 1.0f) {
//         duty_cycle = 1.0f;
//     } else if (duty_cycle < 0.0f) {
//         duty_cycle = 0.0f;
//     }
//     drivers->pwm.write(duty_cycle, GRABBER_PIN);
// }

void GrabberSubsystem::end() {
    activate(0.0f);
    activate(1.0f);
}



} //namespace src::Grabber


#endif //GRABBER_COMPATIBLE