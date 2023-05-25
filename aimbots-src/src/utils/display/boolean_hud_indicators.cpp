#include "boolean_hud_indicators.hpp"

#include <tuple>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command_scheduler.hpp"

using namespace tap::communication::serial;
using namespace tap::communication::referee;
using namespace std;
using namespace src::Hopper;

namespace src::utils::display {
template <RefSerial::Tx::GraphicColor ON_COLOR, RefSerial::Tx::GraphicColor OFF_COLOR>
static inline void updateGraphicColor(bool indicatorStatus, RefSerialData::Tx::Graphic1Message *graphic) {
    graphic->graphicData.color = static_cast<uint32_t>(indicatorStatus ? ON_COLOR : OFF_COLOR) & 0xf;
}

BooleanHudIndicator::BooleanHudIndicator(
    tap::control::CommandScheduler &commandScheduler,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    HopperSubsystem *hopper)
    : HudIndicator(refSerialTransmitter),
      commandScheduler(commandScheduler),
      hopper(hopper),
      booleanHudIndicatorDrawers{
          BooleanHUDIndicator(
              refSerialTransmitter,
              &booleanHudIndicatorGraphics[HOPPER_STATUS],
              updateGraphicColor<
                  get<1>(BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[HOPPER_STATUS]),
                  get<2>(BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[HOPPER_STATUS])>,
              0),
          BooleanHUDIndicator(
              refSerialTransmitter,
              &booleanHudIndicatorGraphics[AGITATOR_STATUS_HEALTHY],
              updateGraphicColor<
                  get<1>(BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[AGITATOR_STATUS_HEALTHY]),
                  get<2>(BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[AGITATOR_STATUS_HEALTHY])>,
              0),
          BooleanHUDIndicator(
              refSerialTransmitter,
              &booleanHudIndicatorGraphics[SPIN_TO_WIN],
              updateGraphicColor<
                  get<1>(BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[SPIN_TO_WIN]),
                  get<2>(BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[SPIN_TO_WIN])>,
              0),
          BooleanHUDIndicator(
              refSerialTransmitter,
              &booleanHudIndicatorGraphics[BOOST_ACTIVE],
              updateGraphicColor<
                  get<1>(BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[BOOST_ACTIVE]),
                  get<2>(BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[BOOST_ACTIVE])>,
              0),
      } {}

modm::ResumableResult<bool> BooleanHudIndicator::sendInitialGraphics() {
    RF_BEGIN(0);

    for (booleanHudIndicatorIndexSendInitialGraphics = 0; booleanHudIndicatorIndexSendInitialGraphics < NUM_BOOLEAN_HUD_INDICATORS;
         booleanHudIndicatorIndexSendInitialGraphics++) {
        RF_CALL(booleanHudIndicatorDrawers[booleanHudIndicatorIndexSendInitialGraphics].initialize());
        RF_CALL(refSerialTransmitter.sendGraphic(&booleanHudIndicatorStaticGraphics[booleanHudIndicatorIndexSendInitialGraphics]));
        RF_CALL(refSerialTransmitter.sendGraphic(&booleanHudIndicatorStaticLabelGraphics[booleanHudIndicatorIndexSendInitialGraphics]));
    }

    RF_END();
}

modm::ResumableResult<bool> BooleanHudIndicator::update() {
    RF_BEGIN(1);

    booleanHudIndicatorDrawers[HOPPER_STATUS].setIndicatorState(hopper->getHopperState() ? true : false);
    booleanHudIndicatorDrawers[AGITATOR_STATUS_HEALTHY].setIndicatorState(true);
    booleanHudIndicatorDrawers[SPIN_TO_WIN].setIndicatorState(false);
    booleanHudIndicatorDrawers[BOOST_ACTIVE].setIndicatorState(false);

    // draw all the booleanHudIndicatorDrawers (only actually sends data if graphic changed)
    for (booleanHudIndicatorIndexUpdate = 0; booleanHudIndicatorIndexUpdate < NUM_BOOLEAN_HUD_INDICATORS; booleanHudIndicatorIndexUpdate++) {
        RF_CALL(booleanHudIndicatorDrawers[booleanHudIndicatorIndexUpdate].draw());
    }

    RF_END();
}

void BooleanHudIndicator::initialize() {
    uint8_t booleanHudIndicatorName[3] = {};
    uint16_t hudIndicatorListCurrY = BOOLEAN_HUD_INDICATOR_LIST_START_Y;

    // Configure hopper cover hud indicator
    for (int i = 0; i < NUM_BOOLEAN_HUD_INDICATORS; i++) {
        // config the boolean HUD indicator circle (that will switch colors based on state)
        getUnusedGraphicName(booleanHudIndicatorName);

        RefSerialTransmitter::configGraphicGenerics(
            &booleanHudIndicatorGraphics[i].graphicData,
            booleanHudIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            get<2>(BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[i]));

        RefSerialTransmitter::configCircle(
            BOOLEAN_HUD_INDICATOR_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            BOOLEAN_HUD_INDICATOR_RADIUS,
            &booleanHudIndicatorGraphics[i].graphicData);

        // config the border circle that bounds the booleanHudIndicatorGraphics
        getUnusedGraphicName(booleanHudIndicatorName);

        RefSerialTransmitter::configGraphicGenerics(
            &booleanHudIndicatorStaticGraphics[i].graphicData,
            booleanHudIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            BOOLEAN_HUD_INDICATOR_OUTLINE_COLOR);

        RefSerialTransmitter::configCircle(
            BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS,
            &booleanHudIndicatorStaticGraphics[i].graphicData);

        // config the label associated with the particular indicator
        getUnusedGraphicName(booleanHudIndicatorName);

        RefSerialTransmitter::configGraphicGenerics(
            &booleanHudIndicatorStaticLabelGraphics[i].graphicData,
            booleanHudIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            BOOLEAN_HUD_INDICATOR_LABEL_COLOR);

        const char *indicatorLabel = get<0>(BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[i]);

        RefSerialTransmitter::configCharacterMsg(
            BOOLEAN_HUD_INDICATOR_LABEL_FONT_SIZE,
            BOOLEAN_HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X - strlen(indicatorLabel) * BOOLEAN_HUD_INDICATOR_LABEL_FONT_SIZE -
                BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS - BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH / 2,
            hudIndicatorListCurrY + BOOLEAN_HUD_INDICATOR_LABEL_FONT_SIZE / 2,
            indicatorLabel,
            &booleanHudIndicatorStaticLabelGraphics[i]);

        // shift the Y pixel location down so the next indicator will be below the indicator was
        // just configured
        hudIndicatorListCurrY -= BOOLEAN_HUD_INDICATOR_LIST_DIST_BETWEEN_BULLETS;
    }
}

}  // namespace src::utils::display