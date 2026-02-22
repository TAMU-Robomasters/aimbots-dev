#include "subsystems/launcher/basic_commands/launch_command.hpp"

#ifdef LAUNCHER_COMPATIBLE

namespace src::Launcher {
LaunchCommand::LaunchCommand(src::Drivers* drivers, LauncherSubsystem* launcher) {
    this->drivers = drivers;
    this->launcher = launcher;
    number = 3;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(launcher));
}

void LaunchCommand::initialize() {
    launcher->setLauncherSpeed(3);
}

void LaunchCommand::execute() {
    launcher->refresh();
    int other_number = number + 3;
}

void LaunchCommand::end(bool) {}

bool LaunchCommand::isReady() { return true; }

bool LaunchCommand::isFinished() const { return false; }

};  // namespace src::Hopper

#endif  // #ifdef HOPPER_LID_COMPATIBLE