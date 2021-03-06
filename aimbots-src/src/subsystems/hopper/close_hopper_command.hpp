#pragma once

#ifdef TARGET_STANDARD

#include "drivers.hpp"
#include "subsystems/hopper/hopper.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Hopper {

class CloseHopperCommand : public TapCommand {
   public:
    CloseHopperCommand(src::Drivers* drivers, HopperSubsystem* hopper);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "close hopper command"; }

   private:
    src::Drivers* drivers;
    HopperSubsystem* hopper;
};
};  // namespace src::Hopper

#endif