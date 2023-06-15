#pragma once

#include <tuple>

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/control/command.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/drivers.hpp"

#include "modm/processing/resumable.hpp"
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/hopper/hopper.hpp"

#include "hud_indicator.hpp"
using namespace src::Hopper;
using namespace src::Chassis;

namespace src::utils::display {
class BooleanHudIndicator : public HudIndicator, protected modm::Resumable<2> {
public:
    BooleanHudIndicator(
        tap::control::CommandScheduler &commandScheduler,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const HopperSubsystem &hopper,
        const ChassisSubsystem &chassis);

    modm::ResumableResult<bool> sendInitialGraphics() override;

    modm::ResumableResult<bool> update() override;

    void initialize() override;

private:
    tap::control::CommandScheduler &commandScheduler;

    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_CENTER_X = 500;  // 500
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_START_Y = 850;   // 760
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_DIST_BETWEEN_BULLETS = 50;

    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_WIDTH = 17;
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_RADIUS = 9;

    static constexpr Tx::GraphicColor BOOLEAN_HUD_INDICATOR_OUTLINE_COLOR = Tx::GraphicColor::BLACK;

    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH = 5;
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS = 20;

    static constexpr Tx::GraphicColor BOOLEAN_HUD_INDICATOR_LABEL_COLOR = Tx::GraphicColor::ORANGE;

    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LABEL_FONT_SIZE = 20;
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH = 3;

    using BooleanHUDIndicatorTuple = std::tuple<const char *, Tx::GraphicColor, Tx::GraphicColor>;

    enum BooleanHUDIndicatorIndex {
        // AGITATOR_STATUS_HEALTHY,
        BOOST_ACTIVE =0,
        SPIN_TO_WIN,
        HOPPER_STATUS,
        NUM_BOOLEAN_HUD_INDICATORS,
    };

    static constexpr BooleanHUDIndicatorTuple BOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[NUM_BOOLEAN_HUD_INDICATORS]{
        // BooleanHUDIndicatorTuple("AGITATOR", Tx::GraphicColor::GREEN, Tx::GraphicColor::PURPLISH_RED),
        BooleanHUDIndicatorTuple("BOOST", Tx::GraphicColor::GREEN, Tx::GraphicColor::PURPLISH_RED),
        BooleanHUDIndicatorTuple("SPINNY", Tx::GraphicColor::GREEN, Tx::GraphicColor::PURPLISH_RED),
        BooleanHUDIndicatorTuple("HOPPER", Tx::GraphicColor::GREEN, Tx::GraphicColor::PURPLISH_RED)};

    Tx::Graphic1Message booleanHudIndicatorGraphics[NUM_BOOLEAN_HUD_INDICATORS];

    tap::communication::referee::BooleanHUDIndicator booleanHudIndicatorDrawers[NUM_BOOLEAN_HUD_INDICATORS];

    int booleanHudIndicatorIndexUpdate = 0;

    int booleanHudIndicatorIndexSendInitialGraphics = 0;

    Tx::Graphic1Message booleanHudIndicatorStaticGraphics[NUM_BOOLEAN_HUD_INDICATORS];
    Tx::GraphicCharacterMessage booleanHudIndicatorStaticLabelGraphics[NUM_BOOLEAN_HUD_INDICATORS];

    const ChassisSubsystem &chassis;
    const HopperSubsystem &hopper;
};

}  // namespace src::utils::display