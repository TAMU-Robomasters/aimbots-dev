#include "subsystems/grabber/grabber.hpp"

#include "tap/communication/gpio/pwm.hpp"

#include "utils/tools/common_types.hpp"
#include "utils/tools/robot_specific_inc.hpp"

//#include "drivers.hpp"

#ifdef GRABBER_COMPATIBLE

#define ON 1
#define OFF 0

/**
 * Potential problem1: on testing, the trigger acting the same to duty cycle = 1 when setting to 0.5. Need more testing on
 * behavior on dudy circle change. Potential problem2: unknown effect on frequency difference in
 * GrabberSubsystem::initialize()
 */
namespace src::Grabber {


GrabberSubsystem::GrabberSubsystem(tap::Drivers* drivers) : Subsystem(drivers), pwm(&drivers->pwm) {}

void GrabberSubsystem::initialize() { pwm->init(); }

void GrabberSubsystem::refresh() {}

void GrabberSubsystem::deactivate() { pwm->write(OFF, GRABBER_PIN); }

void GrabberSubsystem::activate() { pwm->write(ON, GRABBER_PIN); }

/**
 * testing for different behavior on different duty cycle
 */
void GrabberSubsystem::unknown() {
    // while(true) {
    //     drivers->pwm.write(1.0f, GRABBER_PIN);
    //     drivers->pwm.write(0.0f, GRABBER_PIN);
    // }
    pwm->write(ON, GRABBER_PIN);
    pwm->write(OFF, GRABBER_PIN);
}

}  // namespace src::Grabber

#endif  // GRABBER_COMPATIBLE