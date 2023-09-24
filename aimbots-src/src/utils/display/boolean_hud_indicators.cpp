#include "boolean_hud_indicators.hpp"

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command_scheduler.hpp"

using namespace tap::communication::serial;
using namespace tap::communication::referee;

namespace src::Utils::ClientDisplay {

template <RefSerial::Tx::GraphicColor ON_COLOR, RefSerial::Tx::GraphicColor OFF_COLOR>
static inline void updateGraphicColor(bool indicatorStatus, RefSerialData::Tx::Graphic1Message *graphic) {
    graphic->graphicData.color = static_cast<uint32_t>(indicatorStatus ? ON_COLOR : OFF_COLOR) & 0xf;
}

BooleanHUDIndicators::BooleanHUDIndicators(
    tap::control::CommandScheduler &commandScheduler,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    // const HopperSubsystem *hopper,
    const ChassisSubsystem &chassis)
    : HudIndicator(refSerialTransmitter),
      commandScheduler(commandScheduler),
    //   hopper(hopper),
      chassis(chassis),
      booleanHUDIndicatorDrawers{
          BooleanHUDIndicator(
              refSerialTransmitter,
              &booleanHUDIndicatorGraphics[SYSTEMS_CALIBRATING],
              updateGraphicColor<
                  get<1>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[SYSTEMS_CALIBRATING]),
                  get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[SYSTEMS_CALIBRATING])>,
              0),
          BooleanHUDIndicator(
              refSerialTransmitter,
              &booleanHUDIndicatorGraphics[TOKYO_STATUS],
              updateGraphicColor<
                  get<1>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[TOKYO_STATUS]),
                  get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[TOKYO_STATUS])>,
              0),
          BooleanHUDIndicator(
              refSerialTransmitter,
              &booleanHUDIndicatorGraphics[HOPPER_STATUS],
              updateGraphicColor<
                  get<1>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[HOPPER_STATUS]),
                  get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[HOPPER_STATUS])>,
              0),
      }  //
{}

modm::ResumableResult<bool> BooleanHUDIndicators::sendInitialGraphics() {
    RF_BEGIN(0);

    for (HUDIndicatorIndexInit = 0; HUDIndicatorIndexUpdate < NUM_BOOLEAN_HUD_INDICATORS; HUDIndicatorIndexInit++) {
        RF_CALL(booleanHUDIndicatorDrawers[HUDIndicatorIndexInit].initialize());
        RF_CALL(refSerialTransmitter.sendGraphic(&booleanHUDIndicatorStaticGraphics[HUDIndicatorIndexInit]));
        RF_CALL(refSerialTransmitter.sendGraphic(&booleanHUDIndicatorStaticLabelGraphics[HUDIndicatorIndexInit]));
    }

    RF_END();
}



bool hopperIndicatorDisplay = false;
// bool spinIndicatorDisplay = false;

modm::ResumableResult<bool> BooleanHUDIndicators::update() {
    RF_BEGIN(1);

    booleanHUDIndicatorDrawers[SYSTEMS_CALIBRATING].setIndicatorState(false);
    booleanHUDIndicatorDrawers[TOKYO_STATUS].setIndicatorState(false);
    booleanHUDIndicatorDrawers[HOPPER_STATUS].setIndicatorState(/*hopper->getHopperState()*/true);

    // hopperIndicatorDisplay = hopper->getHopperState();

    // draw all the booleanHudIndicatorDrawers (only actually sends data if graphic changed)
    for (HUDIndicatorIndexUpdate = 0; HUDIndicatorIndexUpdate < NUM_BOOLEAN_HUD_INDICATORS; HUDIndicatorIndexUpdate++) {
        RF_CALL(booleanHUDIndicatorDrawers[HUDIndicatorIndexUpdate].draw());
    }

    RF_END();
}

void BooleanHUDIndicators::initialize() {
    uint8_t booleanHUDIndicatorName[3] = {};
    uint16_t hudIndicatorListCurrY = BOOLEAN_HUD_INDICATOR_LIST_START_Y;

    // Configure hopper cover hud indicator
    for (int i = 0; i < NUM_BOOLEAN_HUD_INDICATORS; i++) {
        // config the boolean HUD indicator circle (that will switch colors based on state)
        getUnusedGraphicName(booleanHUDIndicatorName);

        RefSerialTransmitter::configGraphicGenerics(
            &booleanHUDIndicatorGraphics[i].graphicData,
            booleanHUDIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[i]));

        RefSerialTransmitter::configCircle(
            BOOLEAN_HUD_INDICATOR_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            BOOLEAN_HUD_INDICATOR_RADIUS,
            &booleanHUDIndicatorGraphics[i].graphicData);

        // config the border circle that bounds the booleanHUDIndicatorGraphics
        getUnusedGraphicName(booleanHUDIndicatorName);

        RefSerialTransmitter::configGraphicGenerics(
            &booleanHUDIndicatorStaticGraphics[i].graphicData,
            booleanHUDIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            BOOLEAN_HUD_INDICATOR_OUTLINE_COLOR);

        RefSerialTransmitter::configCircle(
            BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS,
            &booleanHUDIndicatorStaticGraphics[i].graphicData);

        // config the label associated with the particular indicator
        getUnusedGraphicName(booleanHUDIndicatorName);

        RefSerialTransmitter::configGraphicGenerics(
            &booleanHUDIndicatorStaticLabelGraphics[i].graphicData,
            booleanHUDIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            BOOLEAN_HUD_INDICATOR_LABEL_COLOR);

        const char *indicatorLabel = get<0>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[i]);

        RefSerialTransmitter::configCharacterMsg(
            BOOLEAN_HUD_INDICATOR_LABEL_FONT_SIZE,
            BOOLEAN_HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X - strlen(indicatorLabel) * BOOLEAN_HUD_INDICATOR_LABEL_FONT_SIZE -
                BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS - BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH / 2,
            hudIndicatorListCurrY + BOOLEAN_HUD_INDICATOR_LABEL_FONT_SIZE / 2,
            indicatorLabel,
            &booleanHUDIndicatorStaticLabelGraphics[i]);

        // shift the Y pixel location down so the next indicator will be below the indicator was
        // just configured
        hudIndicatorListCurrY -= BOOLEAN_HUD_INDICATOR_LIST_DIST_BETWEEN_BULLETS;
    }
}

}  // namespace src::Utils::ClientDisplay