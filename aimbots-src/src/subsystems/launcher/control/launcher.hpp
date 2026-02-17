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

    // motor control commands here, specific open/close commands separate?
    // need to know more about servo class

    // taproot takes 0-1 which I'm assuming maps to 0-360 degrees
    // want to take 0-360 here and map to taproot 0-1

    /**
     * @brief Sets angle for hopper servo to turn to and maintain (don't call continuously!!!)
     *
     */
    void setLauncherSpeed(float launchSpeed);

private:
    tap::Drivers* drivers;

    Servo launcherMotor;
    uint32_t actionStartTime;  // milliseconds
};
};  // namespace src::Hopper

#endif  // #ifdef HOPPER_LID_COMPATIBLE