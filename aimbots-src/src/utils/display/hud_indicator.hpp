#pragma once

#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "modm/processing/resumable.hpp"

namespace src::utils::display {
class HudIndicator : protected tap::communication::serial::RefSerialData {
public:
    static constexpr uint16_t SCREEN_WIDTH = 1920;
    static constexpr uint16_t SCREEN_HEIGHT = 1080;

    static constexpr uint8_t DEFAULT_GRAPHIC_LAYER = 0;

    static constexpr float CHARACTER_LINE_SPACING = 1.5f;

    HudIndicator(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    virtual modm::ResumableResult<bool> sendInitialGraphics() = 0;

    virtual modm::ResumableResult<bool> update() = 0;

    virtual void initialize() = 0;

    static void resetGraphicNameGenerator();

protected:
    static void getUnusedGraphicName(uint8_t graphicName[3]);

    static uint32_t currGraphicName;

    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter;

    tap::arch::MilliTimeout delayTimer;
};

}  // namespace utils::display