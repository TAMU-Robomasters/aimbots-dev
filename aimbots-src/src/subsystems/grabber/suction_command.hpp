#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/grabber/grabber.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#include "grabber.hpp"

#ifdef GRABBER_COMPATIBLE

namespace src:: Grabber {

class SuctionCommand : public TapCommand {
public:
    SuctionCommand(src::Drivers* drivers, GrabberSubsystem* grabber);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "suction command"; }

private:
    src::Drivers* drivers;
    GrabberSubsystem* grabber;

};

}  // namespace src::Grabber

#endif //#ifdef GRABBER_COMPATIBLE