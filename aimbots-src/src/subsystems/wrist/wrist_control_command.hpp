#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/wrist/wrist.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef WRIST_COMPATIBLE

using namespace src::Utils::motion;

namespace src::Wrist {
class WristControlCommand : public TapCommand {
public:
    WristControlCommand(src::Drivers* drivers, WristSubsystem* wrist);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override { return true; };
    bool isFinished() const override { return false; };
    const char* getName() const override { return "control wrist command"; }

private:
    src::Drivers* drivers;
    WristSubsystem* wrist;

    float yawTarget = 0;
    float pitchTarget = 0;
    float rollTarget = 0;

    MilliTimeout joystickReadDelay;
};

};      // namespace src::Wrist
#endif  // #ifdef WRIST_COMPATIBLE
