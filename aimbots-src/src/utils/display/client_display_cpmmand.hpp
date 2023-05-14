
#pragma once

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
//
#include "modm/math/geometry/point_set_2d.hpp"
#include "modm/math/utils/misc.hpp"
#include "modm/processing/protothread.hpp"
//
#include "client_display_subsystem.hpp"

namespace src::utils::display {

class ClientDisplayCommand : public tap::control::Command, ::modm::pt::Protothread {
public:
    ClientDisplayCommand(
        tap::Drivers &drivers,
        tap::control::CommandScheduler &commandScheduler,
        ClientDisplaySubsystem &clientDisplay
        // the rest of the commands
    );

    const char *getName() const override { return "client display"; }

    void initialize() override;

    void execute() override;

    void end(bool) override {}

    bool isFinished() const override { return false; }
};

}  // namespace src::utils::display