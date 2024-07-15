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

namespace src::Utils::ClientDisplay {
class BooleanHUDIndicators : public HudIndicator, protected modm::Resumable<2> {
public:
    BooleanHUDIndicators(
        tap::control::CommandScheduler &commandScheduler,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
         const HopperSubsystem *hopper,
        const ChassisSubsystem &chassis);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

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
        SYSTEMS_CALIBRATING = 0,
        TOKYO_STATUS,
        HOPPER_STATUS,
        NUM_BOOLEAN_HUD_INDICATORS,
    };

    static constexpr BooleanHUDIndicatorTuple BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[NUM_BOOLEAN_HUD_INDICATORS]{
        BooleanHUDIndicatorTuple("CALIB", Tx::GraphicColor::PURPLISH_RED, Tx::GraphicColor::GREEN),
        BooleanHUDIndicatorTuple("TOKYO", Tx::GraphicColor::GREEN, Tx::GraphicColor::PINK),
        BooleanHUDIndicatorTuple("HOPPER", Tx::GraphicColor::GREEN, Tx::GraphicColor::PINK),
    };

     const HopperSubsystem *hopper;
    const ChassisSubsystem &chassis;

    Tx::Graphic1Message booleanHUDIndicatorGraphics[NUM_BOOLEAN_HUD_INDICATORS];

    tap::communication::referee::BooleanHUDIndicator booleanHUDIndicatorDrawers[NUM_BOOLEAN_HUD_INDICATORS];

    int HUDIndicatorIndexUpdate = 0;
    int HUDIndicatorIndexInit = 0;

    Tx::Graphic1Message booleanHUDIndicatorStaticGraphics[NUM_BOOLEAN_HUD_INDICATORS];
    Tx::GraphicCharacterMessage booleanHUDIndicatorStaticLabelGraphics[NUM_BOOLEAN_HUD_INDICATORS];
};

}  // namespace src::Utils::ClientDisplay