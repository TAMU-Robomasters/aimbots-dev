#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/launcher/launcher_constants.hpp"
#include "utils/tools/common_types.hpp"

#ifdef LAUNCHER_COMPATIBLE

namespace src::Launcher {

class LauncherSubsystem : public tap::control::Subsystem {
public:
    LauncherSubsystem(tap::Drivers* drivers);

    mockable void initialize() override;
    void refresh() override;

    void setLauncherSpeed(float launchSpeed);

private:
    tap::Drivers* drivers;

    float launcherPwmTarget = 0.0f;
    uint32_t actionStartTime = 0;  // milliseconds
};

}  // namespace src::Launcher

#endif  // LAUNCHER_COMPATIBLE
