#include "subsystems/launcher/basic_commands/launch_command.hpp"

#ifdef LAUNCHER_COMPATIBLE

namespace src::Launcher {

extern float launcherPwmDisplay;
float launchCommandPwmDisplay = 0.0f;

LaunchCommand::LaunchCommand(src::Drivers* drivers, LauncherSubsystem* launcher)
    : drivers(drivers), launcher(launcher) {
    addSubsystemRequirement(launcher);
}

void LaunchCommand::initialize() {
    launcher->setLauncherSpeed(30.0f);
}

void LaunchCommand::execute() {
    launcher->setLauncherSpeed(30.0f);
    launcher->refresh();
    launchCommandPwmDisplay = launcherPwmDisplay;
}

void LaunchCommand::end(bool) {}

bool LaunchCommand::isReady() { return true; }

bool LaunchCommand::isFinished() const { return false; }

}  // namespace src::Launcher

#endif  // LAUNCHER_COMPATIBLE
