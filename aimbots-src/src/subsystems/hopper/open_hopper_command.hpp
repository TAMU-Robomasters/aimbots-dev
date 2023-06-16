#pragma once
#include "tap/control/subsystem.hpp"

#include "subsystems/hopper/hopper.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Hopper {

class OpenHopperCommand : public TapCommand {
public:
    OpenHopperCommand(src::Drivers* drivers, HopperSubsystem* hopper, float HOPPER_OPEN_ANGLE);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "open hopper command"; }

private:
    src::Drivers* drivers;
    HopperSubsystem* hopper;

    float HOPPER_OPEN_ANGLE;
};
};  // namespace src::Hopper
