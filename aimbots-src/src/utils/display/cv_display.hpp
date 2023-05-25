#pragma once
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command_scheduler.hpp"
// #include "tap/drivers.hpp"
#include <tuple>

#include "tap/control/command.hpp"
#include "tap/control/command_scheduler.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace src::utils::display {
class CVDisplay : public HudIndicator, protected modm::Resumable<2> {
public:
    //TODO: need to add the cv pipe line when i get it and the actual darwings
    CVDisplay(tap::control::CommandScheduler &commandScheduler, tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);
    modm::ResumableResult<bool> sendInitialGraphics() override;

    modm::ResumableResult<bool> update() override;
    void initialize() override;

private:
    static constexpr uint16_t CV_LIST_CENTER_X = 100;
    static constexpr uint16_t CV_LIST_START_Y = 760;
    static constexpr uint16_t CV_LIST_DIST_BETWEEN_BULLETS = 50;

    static constexpr uint16_t CV_WIDTH = 17;
    static constexpr uint16_t CV_RADIUS = 9;

    static constexpr Tx::GraphicColor CV_OUTLINE_COLOR = Tx::GraphicColor::BLACK;

    static constexpr uint16_t CV_OUTLINE_WIDTH = 5;
    static constexpr uint16_t CV_OUTLINE_RADIUS = 20;

    static constexpr Tx::GraphicColor CV_LABEL_COLOR = Tx::GraphicColor::ORANGE;

    static constexpr uint16_t CV_LABEL_FONT_SIZE = 15;
    static constexpr uint16_t CV_LABEL_CHAR_LINE_WIDTH = 3;

    tap::control::CommandScheduler &commandScheduler;
};

}  // namespace src::utils::display