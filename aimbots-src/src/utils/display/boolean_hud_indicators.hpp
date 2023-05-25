#pragma once

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"
// #include "tap/drivers.hpp"
#include <tuple>

#include "tap/control/command.hpp"
#include "tap/control/command_scheduler.hpp"

#include "modm/processing/resumable.hpp"
#include "subsystems/hopper/hopper.hpp"

#include "hud_indicator.hpp"
using namespace src::Hopper;

namespace src::utils::display {
class BooleanHudIndicator : public HudIndicator, protected modm::Resumable<2> {
public:
    BooleanHudIndicator(
        tap::control::CommandScheduler &commandScheduler,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        HopperSubsystem *hopper);

    modm::ResumableResult<bool> sendInitialGraphics() override;

    modm::ResumableResult<bool> update() override;

    void initialize() override;

private:
    tap::control::CommandScheduler &commandScheduler;

    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_CENTER_X = 100;
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_START_Y = 760;
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_DIST_BETWEEN_BULLETS = 50;

    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_WIDTH = 17;
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_RADIUS = 9;

    static constexpr Tx::GraphicColor BOOLEAN_HUD_INDICATOR_OUTLINE_COLOR = Tx::GraphicColor::BLACK;

    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH = 5;
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS = 20;

    static constexpr Tx::GraphicColor BOOLEAN_HUD_INDICATOR_LABEL_COLOR = Tx::GraphicColor::ORANGE;

    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LABEL_FONT_SIZE = 15;
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH = 3;

    using BooleanHUDIndicatorTuple = std::tuple<const char *, Tx::GraphicColor, Tx::GraphicColor>;

    enum BooleanHUDIndicatorIndex {
        HOPPER_STATUS = 0,
        AGITATOR_STATUS_HEALTHY,
        SPIN_TO_WIN,
        NUM_BOOLEAN_HUD_INDICATORS,
    };

    static constexpr BooleanHUDIndicatorTuple BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[NUM_BOOLEAN_HUD_INDICATORS]{
        BooleanHUDIndicatorTuple("HOPPER STATUS", Tx::GraphicColor::WHITE, Tx::GraphicColor::BLACK),
        BooleanHUDIndicatorTuple("AGITATOR STATUS HEALTHY", Tx::GraphicColor::GREEN, Tx::GraphicColor::PURPLISH_RED),
        BooleanHUDIndicatorTuple("SENTRY DRIVE STATUS", Tx::GraphicColor::GREEN, Tx::GraphicColor::PURPLISH_RED),
    };

    Tx::Graphic1Message booleanHudIndicatorGraphics[NUM_BOOLEAN_HUD_INDICATORS];

    tap::communication::referee::BooleanHUDIndicator booleanHudIndicatorDrawers[NUM_BOOLEAN_HUD_INDICATORS];

    int booleanHudIndicatorIndexUpdate = 0;

    int booleanHudIndicatorIndexSendInitialGraphics = 0;

    Tx::Graphic1Message booleanHudIndicatorStaticGraphics[NUM_BOOLEAN_HUD_INDICATORS];
    Tx::GraphicCharacterMessage booleanHudIndicatorStaticLabelGraphics[NUM_BOOLEAN_HUD_INDICATORS];

    HopperSubsystem *hopper;
};

}  // namespace src::utils::display