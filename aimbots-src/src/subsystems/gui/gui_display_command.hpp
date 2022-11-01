#pragma once

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command.hpp"

#include "modm/processing/protothread.hpp"

#include <drivers.hpp>
#include <subsystems/gui/gui_display.hpp>

#include "subsystemStatusGraphic.hpp"

namespace src::GUI {

//According to ARUW, only one Display Command should be instantiated, else unspeakable horrors occur

class GUI_DisplayCommand : public tap::control::Command, ::modm::pt::Protothread {
    public:
        GUI_DisplayCommand(src::Drivers* drivers, GUI_DisplaySubsystem* GUIDisplay);

    void initialize() override;

    void execute() override;

    void end(bool) override {}

    bool isFinished() const override { return false; }

    const char *getName() const override { return "client display"; }
    
    private:
        src::Drivers &drivers;
        tap::communication::serial::RefSerialTransmitter refSerialTransmitter;

        SubsystemStatusGraphic subsystemStatGraphics;

        bool run();
};
}