#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/grabber/grabber.hpp"
#include "utils/common_types.hpp"


#include "drivers.hpp"

#include "grabber.hpp"

#ifdef GRABBER_COMPATIBLE

namespace src:: Grabber {

class Suction_Command : public TapCommand {
public:
    Suction_Command(tap::Drivers* drivers, GrabberSubsystem* grabber);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "suction command"; }

private:
    tap::Drivers* drivers;
    GrabberSubsystem* grabber;

};

}  // namespace src::Grabber

#endif //#ifdef GRABBER_COMPATIBLE