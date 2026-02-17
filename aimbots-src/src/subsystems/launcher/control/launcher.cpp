#include "subsystems/launcher/control/launcher.hpp"

#include "tap/communication/gpio/pwm.hpp"

#include "utils/tools/common_types.hpp"


#include "drivers.hpp"

#ifdef LAUNCHER_COMPATIBLE

namespace src::Launcher {

LauncherSubsystem::LauncherSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      drivers(drivers),
      launcherMotor(drivers, LAUNCHER_PIN, LAUNCHER_MAX_PWM, LAUNCHER_MIN_PWM, LAUNCHER_PWM_LAUNCH_SPEED) {}

void LauncherSubsystem::initialize() {
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 330);  // Timer 1 for C1 Pin
    setLauncherSpeed(3);
}
int number = 4;

void LauncherSubsystem::refresh() { 
    launcherMotor.updateSendPwmRamp(); 
    number = 3;
}


void LauncherSubsystem::setLauncherSpeed(float launchSpeed) {
    launcherMotor.setTargetPwm(35);
    actionStartTime = tap::arch::clock::getTimeMilliseconds();
}
};  // namespace src::Hopper

#endif  // #ifdef HOPPER_LID_COMPATIBLE