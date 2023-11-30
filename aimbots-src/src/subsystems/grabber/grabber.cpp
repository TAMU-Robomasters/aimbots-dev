#include "subsystems/grabber/grabber.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef GRABBER_COMPATIBLE
/**
 * Potential problem1: on testing, the trigger acting the same to duty cycle = 1 when setting to 0.5. Need more testing on behavior on dudy circle change.
 * Potential problem2: unknown effect on frequency difference in GrabberSubsystem::initialize()
 */
namespace src::Grabber {

GrabberSubsystem::GrabberSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers)
{

}
int account_entering_commands;
/**
 * initialize local frequency
 */
void GrabberSubsystem::initialize() {
    drivers->pwm.init();
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1 ,2000);
}

void GrabberSubsystem::refresh() {
}

/**
 * set duty cycle to 0, deactivate the trigger
 */
void GrabberSubsystem::deactivate() {
    drivers->pwm.write(0.0f, GRABBER_PIN);
}


/**
 * set duty cycle to 1, activate the trigger
 */
void GrabberSubsystem::activate(){
    drivers->pwm.write(1.0f, GRABBER_PIN);
}

/**
 * testing for different behavior on different duty cycle
 */
void GrabberSubsystem::unknown(){
    // while(true) {
    //     drivers->pwm.write(1.0f, GRABBER_PIN);
    //     drivers->pwm.write(0.0f, GRABBER_PIN);
    // }
    drivers->pwm.write(1.0f, GRABBER_PIN);
    drivers->pwm.write(0.0f, GRABBER_PIN);
}



} //namespace src::Grabber


#endif //GRABBER_COMPATIBLE