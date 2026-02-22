#include "subsystems/launcher/control/launcher.hpp"

#include "tap/communication/gpio/pwm.hpp"

#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

#ifdef LAUNCHER_COMPATIBLE

namespace src::Launcher {

float launcherPwmDisplay = 0.0f;

LauncherSubsystem::LauncherSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      drivers(drivers) {}

void LauncherSubsystem::initialize() {
    drivers->pwm.init();
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 330);  // Timer 1 for C1 Pin
    setLauncherSpeed(3);
    refresh();
}

void LauncherSubsystem::refresh() {
    launcherPwmDisplay = launcherPwmTarget;
}
void LauncherSubsystem::setLauncherSpeed(float launchSpeed) {
    launcherPwmTarget = launchSpeed;
    float num = 0.0f;
    while(num <= launchSpeed){
        drivers->pwm.write(num, LAUNCHER_PIN);
        num += 0.5;
    }
    actionStartTime = tap::arch::clock::getTimeMilliseconds();
}

}  // namespace src::Launcher

#endif  // LAUNCHER_COMPATIBLE
