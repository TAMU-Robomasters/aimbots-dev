#pragma once

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "modm/processing/resumable.hpp"


#include "graphic_helper.hpp"
#include "drivers.hpp"

namespace src
{
class Drivers;
}

namespace src::GUI {

class SubsystemStatusGraphic : public GraphicHelper, protected modm::Resumable<2> {

public:

SubsystemStatusGraphic(src::Drivers &drivers, tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

modm::ResumableResult<bool> sendInitialGraphics() override final;
modm::ResumableResult<bool> update() override final;
void initialize() override final;

private:

src::Drivers *drivers;

static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_CENTER_X = 280;
static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_START_Y = 760;
static constexpr uint16_t BOOLEAN_HUD_INDICATOR_WIDTH = 17;

Tx::Graphic1Message statusStaticGraphics[2];
Tx::GraphicCharacterMessage statusStaticLabelGraphics[2];

const char * sampleMsg = "Howdy World";


};
}