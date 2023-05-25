
#pragma once
#include <array>
#include <tuple>

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
#include "subsystems/hopper/hopper.hpp"

#include "boolean_hud_indicators.hpp"
#include "client_display_subsystem.hpp"
#include "reticle_indicator.hpp"
using namespace src::Hopper;

namespace src::utils::display {

class ClientDisplayCommand : public tap::control::Command, ::modm::pt::Protothread {
public:
    ClientDisplayCommand(
        tap::Drivers &drivers,
        tap::control::CommandScheduler &commandScheduler,
        ClientDisplaySubsystem &clientDisplay,
        HopperSubsystem *hopper);

    const char *getName() const override { return "client display"; }

    void initialize() override;

    void execute() override;

    void end(bool) override {}

    bool isFinished() const override { return false; }

private:
    tap::Drivers &drivers;
    tap::control::CommandScheduler &commandScheduler;
    // ClientDisplaySubsystem &clientDisplay;
    tap::communication::serial::RefSerialTransmitter refSerialTransmitter;
    BooleanHudIndicator booleanHudIndicator;
    ReticleIndicator reticleIndicator;
    bool run();

    HopperSubsystem *hopper;
};

}  // namespace src::utils::display