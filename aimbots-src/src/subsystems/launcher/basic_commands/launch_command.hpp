#pragma once

#include "subsystems/launcher/control/launcher.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

#ifdef LAUNCHER_COMPATIBLE

namespace src::Launcher {

class LaunchCommand : public TapCommand {
public:
    LaunchCommand(src::Drivers* drivers, LauncherSubsystem* launcher);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "launcher command"; }

private:
    src::Drivers* drivers;
    LauncherSubsystem* launcher;
};

}  // namespace src::Launcher

#endif  // LAUNCHER_COMPATIBLE
