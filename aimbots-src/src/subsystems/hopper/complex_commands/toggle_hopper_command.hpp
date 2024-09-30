#pragma once

#include "subsystems/hopper/control/hopper.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef HOPPER_LID_COMPATIBLE

namespace src::Hopper {

class ToggleHopperCommand : public TapCommand {
public:
    ToggleHopperCommand(src::Drivers* drivers, HopperSubsystem* hopper, float HOPPER_CLOSED_ANGLE, float HOPPER_OPEN_ANGLE);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "toggle hopper command"; }

private:
    src::Drivers* drivers;
    HopperSubsystem* hopper;

    bool wasCPressed = false;
    bool hopperClosed = true;

    float HOPPER_CLOSED_ANGLE;
    float HOPPER_OPEN_ANGLE;
};
}  // namespace src::Hopper

#endif  // #ifdef HOPPER_LID_COMPATIBLE